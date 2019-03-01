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

#ifndef RTR_MOVEIT_ROADMAP_VISUALIZATION_H
#define RTR_MOVEIT_ROADMAP_VISUALIZATION_H

#include <ros/ros.h>
#include <moveit/macros/class_forward.h>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <rtr_moveit/rtr_datatypes.h>

namespace rtr_moveit
{
MOVEIT_CLASS_FORWARD(RoadmapVisualization);
class RoadmapVisualization
{
public:
  RoadmapVisualization(const ros::NodeHandle& nh);

  void visualizeRoadmap(const std::string& frame_id, const geometry_msgs::Pose& marker_pose,
                        const std::vector<geometry_msgs::Point>& state_positions,
                        const std::vector<geometry_msgs::Point>& state_edges = {});

  void visualizeSolutionPath(const std::string& frame_id, const geometry_msgs::Pose& marker_pose,
                             const std::vector<geometry_msgs::Point>& waypoints);

  void visualizeVolumeRegion(const RoadmapVolume& volume);

  void visualizeOccupancy(const RoadmapVolume& volume, const OccupancyData& occupancy_data);

private:
  ros::Publisher marker_pub_;
  double marker_lifetime_;
  ros::NodeHandle nh_;
};
}  // namespace rtr_moveit

#endif  // RTR_MOVEIT_ROADMAP_VISUALIZATION_H
