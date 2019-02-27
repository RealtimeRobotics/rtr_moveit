/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, PickNik LLC
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
 * Desc: Generation of occupancy data from pcl or planning scenes
 */

#include <rtr_moveit/occupancy_handler.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <chrono>
#include <exception>
#include <stdexcept>

// Eigen
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

// collision checks
#include <moveit/collision_detection/world.h>
#include <moveit/collision_detection_fcl/collision_world_fcl.h>
#include <geometric_shapes/shapes.h>

// RapidPlan
#include <rtr-occupancy/Voxel.hpp>

namespace rtr_moveit
{
const std::string LOGNAME = "occupancy_handler";
OccupancyHandler::OccupancyHandler(const ros::NodeHandle& nh) : nh_(nh)
{
}

void OccupancyHandler::setVolumeRegion(const RoadmapVolume& roadmap_volume)
{
  volume_region_ = roadmap_volume;
}

void OccupancyHandler::setPointCloudTopic(const std::string& pcl_topic)
{
  if (pcl_topic != pcl_topic_)
  {
    pcl_topic_ = pcl_topic;
    pcl_sub_.shutdown();
    pcl_sub_ = nh_.subscribe(pcl_topic_, 1, &OccupancyHandler::pclCallback, this);
  }
}

bool OccupancyHandler::fromPointCloud(OccupancyData& occupancy_data, double timeout)
{
  // wait for new point cloud message
  const auto start_time = ros::Time::now();
  while (true)
  {
    if ((ros::Time::now() - start_time).toSec() > timeout)
    {
      ROS_WARN_STREAM_NAMED(LOGNAME, "Timeout waiting for point cloud data on topic: " << pcl_topic_);
      return false;
    }
    if (next_cloud_)
    {
      // only use point cloud if it's younger than 100ms, otherwise wait for a new one
      ros::Time pcl_stamp;
      pcl_conversions::fromPCL(next_cloud_->header.stamp, pcl_stamp);
      if ((start_time - pcl_stamp).toNSec() > (100 * 1e+6))
        ROS_WARN_THROTTLE_NAMED(10, LOGNAME, "Point cloud data is too far in the past");
      else
        break;  // success
    }
  }

  // convert cloud to pcl::PointCloud
  pcl::PCLPointCloud2ConstPtr cloud_pcl2 = next_cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromPCLPointCloud2(*cloud_pcl2, *cloud);
  // lookup cloud_to_volume transform
  tf::StampedTransform cloud_to_volume;
  try
  {
    tf_listener_.lookupTransform(volume_region_.pose.header.frame_id, cloud->header.frame_id, ros::Time(0),
                                 cloud_to_volume);
  }
  catch (const tf2::TransformException& e)
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Exception when trying to to lookup transform: " << e.what());
    return false;
  }
  // transform cloud to volume frame
  pcl_ros::transformPointCloud(*cloud, *cloud, cloud_to_volume);
  occupancy_data.point_cloud = cloud;

  // get result
  occupancy_data.type = OccupancyData::Type::POINT_CLOUD;
  return occupancy_data.point_cloud != NULL;
}

void OccupancyHandler::pclCallback(const pcl::PCLPointCloud2ConstPtr& cloud_pcl2)
{
  next_cloud_ = cloud_pcl2;
}

bool OccupancyHandler::fromPlanningScene(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                         OccupancyData& occupancy_data)
{
  // region volume dimensions
  float x_length = volume_region_.dimension[0];
  float y_length = volume_region_.dimension[1];
  float z_length = volume_region_.dimension[2];

  // voxel resolution
  float x_voxels = volume_region_.voxel_resolution[0];
  float y_voxels = volume_region_.voxel_resolution[1];
  float z_voxels = volume_region_.voxel_resolution[2];

  // voxel dimensions
  float x_voxel_dimension = x_length / x_voxels;
  float y_voxel_dimension = y_length / y_voxels;
  float z_voxel_dimension = z_length / z_voxels;

  // Compute transform: world->volume
  // world_to_volume points at the corner of the volume origin (x=0,y=0,z=0)
  // we use auto to support Affine3d and Isometry3d (kinetic + melodic)
  auto world_to_base(planning_scene->getFrameTransform(volume_region_.pose.header.frame_id));
  auto base_to_volume = world_to_base;
  tf::poseMsgToEigen(volume_region_.pose.pose, base_to_volume);
  auto world_to_volume = world_to_base * base_to_volume;

  // create collision world and add voxel box shape one step outside the volume grid
  collision_detection::CollisionWorldFCL world;
  shapes::Box box(x_voxel_dimension, y_voxel_dimension, z_voxel_dimension);
  Eigen::Translation3d box_start_position(-0.5 * x_voxel_dimension, -0.5 * y_voxel_dimension, -0.5 * z_voxel_dimension);

  // occupancy box id and dimensions
  // TODO(henningkayser): Check that box id is not present in planning scene - should be unique
  std::string box_id = "rapidplan_collision_box";
  world.getWorld()->addToObject(box_id, std::make_shared<const shapes::Box>(box), world_to_volume * box_start_position);

  // clear scene boxes vector
  occupancy_data.type = OccupancyData::Type::VOXELS;
  occupancy_data.voxels.resize(0);

  // x/y/z translation steps, since relative movements are more efficient than repositioning the object
  auto volume_orientation = world_to_volume.rotation();
  auto x_step(volume_orientation * decltype(world_to_volume)(Eigen::Translation3d(x_voxel_dimension, 0, 0)));
  auto y_step(volume_orientation * decltype(world_to_volume)(Eigen::Translation3d(0, y_voxel_dimension, 0)));
  auto z_step(volume_orientation * decltype(world_to_volume)(Eigen::Translation3d(0, 0, z_voxel_dimension)));

  // x/y reset transforms
  auto y_reset(volume_orientation *
               decltype(world_to_volume)(Eigen::Translation3d(0, -y_voxels * y_voxel_dimension, 0)));
  auto z_reset(volume_orientation *
               decltype(world_to_volume)(Eigen::Translation3d(0, 0, -z_voxels * z_voxel_dimension)));

  // Loop over X/Y/Z voxel positions and check for box collisions in the collision scene
  // NOTE: This implementation is a prototype and will be replaced by more efficient methods as described below
  // TODO(RTR-57): More efficient implementations:
  //                          * Iterate over collision objects and only sample local bounding boxes
  //                          * Use octree search, since boxes can have variable sizes
  // TODO(RTR-57): adjust grid to odd volume dimensions
  // TODO(RTR-57): Do we need extra Box padding here?
  collision_detection::CollisionRequest request;
  collision_detection::CollisionResult result;
  for (uint16_t x = 0; x < x_voxels; ++x)
  {
    world.getWorld()->moveObject(box_id, x_step);
    for (uint16_t y = 0; y < y_voxels; ++y)
    {
      world.getWorld()->moveObject(box_id, y_step);
      for (uint16_t z = 0; z < z_voxels; ++z)
      {
        world.getWorld()->moveObject(box_id, z_step);
        planning_scene->getCollisionWorld()->checkWorldCollision(request, result, world);
        if (result.collision)
        {
          occupancy_data.voxels.push_back(rtr::Voxel(x, y, z));
          result.clear();  // TODO(RTR-57): Is this really necessary?
        }
      }
      // move object back to z start
      world.getWorld()->moveObject(box_id, z_reset);
    }
    // move object back to y start
    world.getWorld()->moveObject(box_id, y_reset);
  }
  return true;
}
}  // namespace rtr_moveit
