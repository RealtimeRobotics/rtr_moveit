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

#ifndef RTR_INTERFACE_RTR_PLANNER_INTERFACE_H
#define RTR_INTERFACE_RTR_PLANNER_INTERFACE_H

#include <deque>
#include <string>
#include <map>

#include <ros/ros.h>

#include <rtrapi/PathPlanner.h>
#include <rtrapi/HardwareInterface.h>

#include <moveit/macros/class_forward.h>
#include <moveit_msgs/RobotState.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

namespace rtr_interface
{
MOVEIT_CLASS_FORWARD(RTRPlannerInterface);

struct RoadmapSpecification
{
  std::string occupancy_file;
  std::string region_file;
  std::string edges_file;
  std::string configs_file;
  std::string transforms_file;
};

class RTRPlannerInterface
{
public:
  RTRPlannerInterface();
  virtual ~RTRPlannerInterface();

  /** \brief Initialize the HardwareInterface */
  bool initialize();

  /** \brief Check if the HardwareInterface is available and the planner can receive requests */
  bool isReady() const;

  /** \brief Run planning attempt and generate a robot trajectory*/
  bool solve(const std::string& group_name, const moveit_msgs::RobotState& start_state,
             const geometry_msgs::Pose goal_pose, robot_trajectory::RobotTrajectory& trajectory);

  // The PathPlanner does not support planning for specific goal states.
  // This behavior could be implemented by searching for the closest existing
  // states and calling FindPath with the corresponding state ids.
  // The solution could then be connected to the goal state by interpolation.
  bool solve(const std::string& group_name, const moveit_msgs::RobotState& start_state,
             const moveit_msgs::RobotState& goal_state, robot_trajectory::RobotTrajectory& trajectory);

protected:
  /** \brief Initialize PathPlanner and HardwareInterface with a given roadmap identifier */
  bool prepareRoadmap(const std::string& roadmap, uint16_t& roadmap_index);

  /** \brief Process waypoints and edges of the solution and create a joint trajectory */
  void processSolutionPath(const std::deque<unsigned int>& waypoints, const std::deque<unsigned int>& edges,
                           robot_trajectory::RobotTrajectory& trajectory) const;

private:
  rtr::HardwareInterface hardware_interface_;
  rtr::PathPlanner planner_;

  // available roadmap specifications
  std::map<std::string, RoadmapSpecification> roadmaps_;
  // name of roadmap loaded by the planner
  std::string loaded_roadmap_;
  // indices of roadmaps written to the board
  std::map<uint16_t, std::string> roadmap_indices_;

  bool findRoadmapIndex(const std::string& roadmap, uint16_t& roadmap_index)
  {
    for (auto it = roadmap_indices_.begin(); it != roadmap_indices_.end(); it++)
    {
      if (it->second == roadmap)
      {
        roadmap_index = it->first;
        return true;
      }
    }
    return false;
  }
};
}  // namespace rtr_interface

#endif  // RTR_INTERFACE_RTR_PLANNER_INTERFACE_H
