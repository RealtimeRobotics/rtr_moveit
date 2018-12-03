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

#include <moveit/planning_interface/planning_interface.h>
#include <pluginlib/class_list_macros.hpp>

#include <rtr_interface/rtr_planning_context.h>

namespace rtr_interface
{
class RTRPlannerManager : public planning_interface::PlannerManager
{
public:
  RTRPlannerManager() : planning_interface::PlannerManager()
  {
  }

  virtual bool initialize(const robot_model::RobotModelConstPtr& model, const std::string& ns)
  {
    ROS_INFO_NAMED("rtr_planner_manager", "Initialized");
    return true;
  }

  virtual bool canServiceRequest(const moveit_msgs::MotionPlanRequest& req) const
  {
    return false;
  }

  virtual std::string getDescription() const
  {
    return "RTR";
  }

  virtual void getPlanningAlgorithms(std::vector<std::string>& algs) const
  {
    algs.push_back("RapidPlan");
  }

  virtual void setPlannerConfigurations(const planning_interface::PlannerConfigurationMap& pconfig)
  {
  }

  virtual planning_interface::PlanningContextPtr
  getPlanningContext(const planning_scene::PlanningSceneConstPtr& planning_scene,
                     const planning_interface::MotionPlanRequest& req, moveit_msgs::MoveItErrorCodes& error_code) const
  {
    RTRPlanningContextPtr ptr;
    return ptr;
  }
};
}  // namespace rtr_interface

PLUGINLIB_EXPORT_CLASS(rtr_interface::RTRPlannerManager, planning_interface::PlannerManager);
