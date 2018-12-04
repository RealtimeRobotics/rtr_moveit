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
Desc: henningkayser@picknik.ai
*/

#ifndef RTR_INTERFACE_RTR_CONVERSIONS_H
#define RTR_INTERFACE_RTR_CONVERSIONS_H

#include <array>
#include <cmath>
#include <deque>
#include <vector>

#include <Eigen/Geometry>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose.h>
#include <trajectory_msgs/JointTrajectory.h>

namespace rtr_interface
{
void poseMsgToRTR(const geometry_msgs::Pose& pose, std::array<float, 6>& rtr_transform)
{
  // set position x/y/z
  rtr_transform[0] = pose.position.x;
  rtr_transform[1] = pose.position.y;
  rtr_transform[2] = pose.position.z;

  // set orientation roll pitch yaw
  Eigen::Quaterniond rotation(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  Eigen::Vector3d euler_angles = rotation.toRotationMatrix().eulerAngles(0, 1, 2);
  rtr_transform[3] = euler_angles[0];
  rtr_transform[4] = euler_angles[1];
  rtr_transform[5] = euler_angles[2];
}

void poseRTRToMsg(const std::array<float, 6>& rtr_transform, geometry_msgs::Pose& pose)
{
  pose.position.x = rtr_transform[0];
  pose.position.y = rtr_transform[1];
  pose.position.z = rtr_transform[2];
  tf::Quaternion rotation = tf::createQuaternionFromRPY(rtr_transform[3], rtr_transform[4], rtr_transform[5]);
  tf::quaternionTFToMsg(rotation, pose.orientation);
}

void pathRTRToJointTrajectory(const std::vector<std::vector<float>>& roadmap_states,
                              const std::deque<unsigned int>& path_indices,
                              trajectory_msgs::JointTrajectory& trajectory)
{
  trajectory.points.resize(path_indices.size());
  int joints_num = roadmap_states[0].size();
  for (auto i : path_indices)
  {
    // fill joint values
    trajectory.points[i].positions.resize(joints_num);
    for (int j = 0; j < joints_num; j++)
      trajectory.points[i].positions[j] = roadmap_states[i][j];

    // set remaining values to 0.0
    trajectory.points[i].velocities.resize(joints_num);
    trajectory.points[i].accelerations.resize(joints_num);
    trajectory.points[i].effort.resize(joints_num);
    trajectory.points[i].time_from_start = ros::Duration(0.0);
  }
}
}  // namespace rtr_interface

#endif  // RTR_INTERFACE_RTR_CONVERSIONS_H
