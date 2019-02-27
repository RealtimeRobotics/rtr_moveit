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
 * Desc: Generation of occupancy data from pcl or planning scenes
 */

#ifndef RTR_MOVEIT_OCCUPANCY_HANDLER_H
#define RTR_MOVEIT_OCCUPANCY_HANDLER_H

// C++ synchronization
#include <tf/transform_listener.h>

// PCL
#include <pcl/pcl_base.h>

// MoveIt!
#include <moveit/planning_scene/planning_scene.h>

// rtr_moveit
#include <rtr_moveit/rtr_datatypes.h>

namespace rtr_moveit
{
class OccupancyHandler
{
public:
  /* @brief Constructor */
  OccupancyHandler(const ros::NodeHandle& nh);

  /* @brief Set the volume region for all occupancy data queries
   * @param  roadmap_volume  - The new region
   */
  void setVolumeRegion(const RoadmapVolume& roadmap_volume);

  /* @brief Set the point cloud topic for all pcl queries
   * @param  pcl_topic  - The new pcl topic
   */
  void setPointCloudTopic(const std::string& pcl_topic);

  /* @brief Initializes occupancy_data with a new point cloud
   * @param  occupancy_data  - the result data including the point cloud
   * @param  timeout - timeout in seconds
   * @return true on success
   */
  bool fromPointCloud(OccupancyData& occupancy_data, double timeout = 1.0);

  /* @brief Generates a list of occupancy voxels given a planning scene
   * @param  planning_scene  - the planning scene
   * @param  occupancy_data  - the result data including the voxels
   * @return true on success
   */
  bool fromPlanningScene(const planning_scene::PlanningSceneConstPtr& planning_scene, OccupancyData& occupancy_data);

private:
  /* Callback function for point cloud subscribers
   * @param  cloud_pcl2 - the pointer of a new sensed point cloud
   */
  void pclCallback(const pcl::PCLPointCloud2ConstPtr& cloud_pcl2);

  ros::NodeHandle nh_;
  RoadmapVolume volume_region_;
  std::string pcl_topic_;

  // PCL synchronization
  pcl::PCLPointCloud2ConstPtr next_cloud_;
  bool pcl_ready_ = false;
  ros::Subscriber pcl_sub_;
  tf::TransformListener tf_listener_;
};
}  // namespace rtr_moveit

#endif  // RTR_MOVEIT_OCCUPANCY_HANDLER_H
