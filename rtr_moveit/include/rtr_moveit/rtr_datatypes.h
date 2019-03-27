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
 * Desc: Structs for roadmap and configuration data types
 */

#ifndef RTR_MOVEIT_RTR_DATATYPES_H
#define RTR_MOVEIT_RTR_DATATYPES_H

#include <set>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <geometric_shapes/shapes.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rtr-occupancy/Voxel.hpp>

namespace rtr_moveit
{
struct RoadmapVolume
{
  // the pose of the origin corner of th volume region (0,0,0)
  geometry_msgs::PoseStamped pose;
  // region volume dimensions along X/Y/Z axes
  std::array<float, 3> dimension;
  // voxel resolution along X/Y/Z axes
  std::array<uint16_t, 3> voxel_resolution;
};

struct RoadmapSpecification
{
  std::string roadmap_id;
  std::string og_file;
  RoadmapVolume volume;

  std::string base_link_frame;
  std::string end_effector_frame;
};

struct OccupancyData
{
  enum Type
  {
    POINT_CLOUD,
    VOXELS
  };
  Type type;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr point_cloud;
  std::vector<rtr::Voxel> voxels;
};

// Configuration for a MoveIt! planning group
struct GroupConfig
{
  std::string group_name;
  std::string default_roadmap_id;
  std::set<std::string> roadmap_ids;
};
}  // namespace rtr_moveit

#endif  // RTR_MOVEIT_RTR_DATATYPES_H
