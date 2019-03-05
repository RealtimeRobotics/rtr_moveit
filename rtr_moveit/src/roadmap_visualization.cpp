/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, PickNik LLC
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik LLC nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Henning Kayser
 * Desc: Visualization of roadmap and planning data
 */

#include <rtr_moveit/roadmap_visualization.h>

namespace rtr_moveit
{
const std::string LOGNAME = "roadmap_visualization";
// marker ids so we don't confuse them
constexpr size_t ROADMAP_STATES_ID = 0;
constexpr size_t ROADMAP_EDGES_ID = 1;
constexpr size_t SOLUTION_PATH_ID = 2;
constexpr size_t VOLUME_REGION_ID = 3;
constexpr size_t VOXELS_ID = 4;

RoadmapVisualization::RoadmapVisualization(const ros::NodeHandle& nh) : nh_(nh)
{
  marker_lifetime_ = nh_.param("planner_config/visualization_marker_lifetime", 0.0);
  if (marker_lifetime_ < 0.0)
  {
    ROS_WARN_NAMED(LOGNAME, "Invalid negative value in parameter visualization_marker_lifetime. Using default: 0.0");
    marker_lifetime_ = 0.0;
  }
  std::string marker_topic =
      nh_.param<std::string>("planner_config/visualization_marker_topic", "/rapidplan_visualization_markers");
  marker_pub_ = nh_.advertise<visualization_msgs::Marker>(marker_topic, 5, false);
  ROS_INFO_STREAM_NAMED(LOGNAME, "Publishing visualization markers to topic: " << marker_topic);
}

void RoadmapVisualization::visualizeRoadmap(const std::string& frame_id, const geometry_msgs::Pose& marker_pose,
                                            const std::vector<geometry_msgs::Point>& state_positions,
                                            const std::vector<geometry_msgs::Point>& state_edges)
{
  if (!state_positions.empty())
  {
    visualization_msgs::Marker roadmap_states;
    roadmap_states.header.frame_id = frame_id;
    roadmap_states.header.stamp = ros::Time::now();
    roadmap_states.id = ROADMAP_STATES_ID;
    roadmap_states.type = visualization_msgs::Marker::SPHERE_LIST;
    roadmap_states.scale.x = 0.005;
    roadmap_states.scale.y = 0.005;
    roadmap_states.scale.z = 0.005;
    roadmap_states.points = state_positions;
    roadmap_states.action = visualization_msgs::Marker::ADD;
    roadmap_states.pose = marker_pose;
    // States are gray
    roadmap_states.color.a = 1.0;
    roadmap_states.color.r = 0.6;
    roadmap_states.color.g = 0.6;
    roadmap_states.color.b = 0.6;
    roadmap_states.lifetime = ros::Duration(marker_lifetime_);
    marker_pub_.publish(roadmap_states);

    if (!state_edges.empty())
    {
      visualization_msgs::Marker roadmap_edges;
      roadmap_edges.header.frame_id = frame_id;
      roadmap_edges.header.stamp = ros::Time::now();
      roadmap_edges.id = ROADMAP_EDGES_ID;
      roadmap_edges.type = visualization_msgs::Marker::LINE_LIST;
      roadmap_edges.scale.x = 0.0005;
      roadmap_edges.points = state_edges;
      roadmap_edges.action = visualization_msgs::Marker::ADD;
      roadmap_edges.pose = marker_pose;
      // State edges are green
      roadmap_edges.color.a = 0.9;
      roadmap_edges.color.g = 0.6;
      roadmap_edges.lifetime = ros::Duration(marker_lifetime_);
      marker_pub_.publish(roadmap_edges);
    }
  }
}

void RoadmapVisualization::visualizeSolutionPath(const std::string& frame_id, const geometry_msgs::Pose& marker_pose,
                                                 const std::vector<geometry_msgs::Point>& waypoint_positions)
{
  visualization_msgs::Marker roadmap_states;
  roadmap_states.header.frame_id = frame_id;
  roadmap_states.header.stamp = ros::Time::now();
  roadmap_states.id = SOLUTION_PATH_ID;
  roadmap_states.type = visualization_msgs::Marker::LINE_STRIP;
  roadmap_states.scale.x = 0.01;
  roadmap_states.points = waypoint_positions;
  roadmap_states.action = visualization_msgs::Marker::ADD;
  roadmap_states.pose = marker_pose;
  // Path is blue
  roadmap_states.color.a = 1.0;
  roadmap_states.color.b = 0.8;
  roadmap_states.lifetime = ros::Duration(marker_lifetime_);
  marker_pub_.publish(roadmap_states);
}

void RoadmapVisualization::visualizeVolumeRegion(const RoadmapVolume& volume)
{
  visualization_msgs::Marker volume_marker;
  volume_marker.header.frame_id = volume.pose.header.frame_id;
  volume_marker.header.stamp = ros::Time::now();
  volume_marker.id = VOLUME_REGION_ID;
  volume_marker.type = visualization_msgs::Marker::CUBE_LIST;
  volume_marker.scale.x = volume.dimension[0];
  volume_marker.scale.y = volume.dimension[1];
  volume_marker.scale.z = volume.dimension[2];
  volume_marker.action = visualization_msgs::Marker::ADD;
  volume_marker.pose = volume.pose.pose;
  volume_marker.points.resize(1);
  volume_marker.points[0].x = 0.5 * volume.dimension[0];
  volume_marker.points[0].y = 0.5 * volume.dimension[1];
  volume_marker.points[0].z = 0.5 * volume.dimension[2];
  // This is a transparent box
  volume_marker.color.a = 0.2;
  volume_marker.color.r = 1.0;
  volume_marker.color.g = 1.0;
  volume_marker.color.b = 1.0;
  volume_marker.lifetime = ros::Duration(marker_lifetime_);
  marker_pub_.publish(volume_marker);
}

void RoadmapVisualization::visualizeOccupancy(const RoadmapVolume& volume, const OccupancyData& occupancy_data)
{
  if (occupancy_data.type != OccupancyData::Type::VOXELS)
  {
    ROS_ERROR("Unable to visualize occupancy data other than VOXELS");
    return;
  }
  if (!occupancy_data.voxels.empty())
  {
    // visualize voxels
    visualization_msgs::Marker voxels;
    voxels.header.frame_id = volume.pose.header.frame_id;
    voxels.header.stamp = ros::Time::now();
    voxels.pose = volume.pose.pose;
    voxels.id = VOXELS_ID;
    voxels.type = visualization_msgs::Marker::CUBE_LIST;
    voxels.action = visualization_msgs::Marker::ADD;
    // Voxels are red
    voxels.color.a = 0.6;
    voxels.color.r = 0.6;
    voxels.lifetime = ros::Duration(marker_lifetime_);

    voxels.scale.x = volume.dimension[0] / volume.voxel_resolution[0];
    voxels.scale.y = volume.dimension[1] / volume.voxel_resolution[1];
    voxels.scale.z = volume.dimension[2] / volume.voxel_resolution[2];

    // generate voxel points in reference to volume region origin pose
    voxels.points.resize(occupancy_data.voxels.size());
    for (std::size_t i = 0; i < voxels.points.size(); i++)
    {
      voxels.points[i].x = (occupancy_data.voxels[i].x + 0.5) * voxels.scale.x;
      voxels.points[i].y = (occupancy_data.voxels[i].y + 0.5) * voxels.scale.y;
      voxels.points[i].z = (occupancy_data.voxels[i].z + 0.5) * voxels.scale.z;
    }
    marker_pub_.publish(voxels);
  }
}
}
