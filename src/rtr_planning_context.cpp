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

#include <string>
#include <vector>

#include <rtr_interface/rtr_planning_context.h>
#include <rtr_interface/rtr_planner_interface.h>

#include <moveit_msgs/Constraints.h>

const std::string LOGNAME = "rtr_planning_context";

namespace rtr_interface
{
// Short helper function to extract a goal pose from goal constraints.
// This will be replaced by more sophisticated methods, that support
// joint states and generate matching goal tolerances and weights.
bool getGoalPose(const std::vector<moveit_msgs::Constraints>& goal_constraints, geometry_msgs::Pose& goal_pose)
{
  if (goal_constraints.size() != 1)
    return false;
  if (goal_constraints[0].position_constraints.size() != 1)
    return false;
  if (goal_constraints[0].position_constraints[0].constraint_region.primitive_poses.size() != 1)
    return false;
  return goal_constraints[0].position_constraints[0].constraint_region.primitive_poses[0];
}

RTRPlanningContext::RTRPlanningContext(const std::string& name, const std::string& group,
                                       const RTRPlannerInterfacePtr& planner_interface)
  : planning_interface::PlanningContext(name, group), planner_interface_(planner_interface)
{
}

bool RTRPlanningContext::solve(planning_interface::MotionPlanResponse& res)
{
  // check planner interface
  if (!planner_interface_->isReady())
  {
    if (!planner_interface_->initialize())
    {
      ROS_ERROR_NAMED(LOGNAME, "Unable to initialize planner!");
      res.error_code_.val = res.error_code_.FAILURE;
      return false;
    }
  }
  ros::Time start_time = ros::Time::now();

  // get goal pose
  geometry_msgs::Pose goal_pose;
  if (!getGoalPose(request_.goal_constraints, goal_pose))
  {
    ROS_ERROR_NAMED(LOGNAME, "Invalid set of goal constraints. Only position goals are supported!");
    res.error_code_.val = res.error_code_.FAILURE;
    return false;
  }

  // start planning attempt
  bool success = planner_interface_->solve(request_.group_name, request_.start_state, goal_pose, *res.trajectory_);

  // fill response
  res.planning_time_ = (ros::Time::now() - start_time).toSec();
  res.error_code_.val = success ? res.error_code_.SUCCESS : res.error_code_.PLANNING_FAILED;
  return success;
}

bool RTRPlanningContext::solve(planning_interface::MotionPlanDetailedResponse& res)
{
  return false;
}

void RTRPlanningContext::clear()
{
}

bool RTRPlanningContext::terminate()
{
  // RapidPlan does not support this right now
  ROS_WARN_STREAM_NAMED(LOGNAME, "Failed to terminate the planning attempt! This is not supported.");
  return false;
}
}  // namespace rtr_interface
