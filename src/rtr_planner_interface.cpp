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

#include <deque>
#include <string>
#include <vector>

#include <rtr_interface/rtr_planner_interface.h>
#include <rtr_interface/rtr_conversions.h>

const std::string LOGNAME = "rtr_planner_interface";

namespace rtr_interface
{
RTRPlannerInterface::RTRPlannerInterface()
{
  // load parameters
}

bool RTRPlannerInterface::initialize()
{
  // check if hardware is connected
  if (!hardware_interface_.Connected())
  {
    ROS_ERROR_NAMED(LOGNAME, "Failed to initialize RapidPlan interface. Hardware is not connected!");
    return false;
  }

  // try to initialize hardware and perform handshake
  if (!(hardware_interface_.Init() && hardware_interface_.Handshake()))
  {
    ROS_ERROR_NAMED(LOGNAME, "Failed to initialize RapidPlan interface. Unable to initialize Hardware!");
    return false;
  }

  ROS_INFO_NAMED(LOGNAME, "RapidPlan interface initialized!");
  return true;
}

bool RTRPlannerInterface::isReady() const
{
  // check connection
  if (!hardware_interface_.Connected())
  {
    ROS_WARN_NAMED(LOGNAME, "RapidPlan interface is not ready. Hardware is not connected!");
    return false;
  }

  // try handshake
  if (!hardware_interface_.Handshake())
  {
    ROS_WARN_NAMED(LOGNAME, "RapidPlan interface is not ready. Hardware is not initialized!");
    return false;
  }

  ROS_DEBUG_NAMED(LOGNAME, "RapidPlan interface is ready!");
  return true;
}

bool RTRPlannerInterface::solve(const std::string& group_name, const moveit_msgs::RobotState& start_state,
                                const geometry_msgs::Pose goal_pose, robot_trajectory::RobotTrajectory& trajectory)
{
  // probably all solve() functions should be threadsave/mutex locked

  // verify roadmap and retrive roadmap index
  uint16_t roadmap_index;
  if (!prepareRoadmap(group_name, roadmap_index))
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Plan aborted!");
    return false;
  }
  // Find closest existing state in roadmap
  unsigned int start_config = 0;  // dummy

  // convert goal pose to Transform
  rtr::Transform goal_transform, tolerance, weights;
  poseMsgToRTR(goal_pose, goal_transform);

  // query collisions from board
  std::vector<uint8_t> collisions;
  std::vector<rtr::Voxel> obstacles;  // dummy
  if (!hardware_interface_.CheckScene(obstacles, roadmap_index, collisions))
  {
    ROS_ERROR_NAMED(LOGNAME, "Hardware Interface failed to check collision scene!");
    return false;
  }

  // query planner
  std::deque<unsigned int> waypoints;
  std::deque<unsigned int> edges;
  int result = planner_.FindPath(start_config, goal_transform, collisions, tolerance, weights, waypoints, edges);
  if (result != 0)
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "RapidPlan failed at finding a valid path - " << planner_.GetError(result));
    return false;
  }

  // create joint trajectory from solution
  processSolutionPath(waypoints, edges, trajectory);
  return true;
}

bool RTRPlannerInterface::solve(const std::string& group_name, const moveit_msgs::RobotState& start_state,
                                const moveit_msgs::RobotState& goal_state,
                                robot_trajectory::RobotTrajectory& trajectory)
{
  return false;
}

bool RTRPlannerInterface::prepareRoadmap(const std::string& roadmap, uint16_t& roadmap_index)
{
  // check if roadmap specification exists
  if (roadmaps_.find(roadmap) == roadmaps_.end())
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "No RoadmapSpecification found for roadmap identifier: " << roadmap);
    return false;
  }

  RoadmapSpecification roadmap_spec = roadmaps_[roadmap];

  // verify that the roadmap is loaded in the PathPlanner
  if (roadmap != loaded_roadmap_)
  {
    if (!planner_.LoadRoadmap(roadmap_spec.configs_file, roadmap_spec.edges_file, roadmap_spec.transforms_file))
    {
      ROS_ERROR_NAMED(LOGNAME, "Unable to load roadmap to PathPlanner!");
      return false;
    }
  }

  // check if roadmap is already written to hardware
  if (!findRoadmapIndex(roadmap, roadmap_index))
  {
    // write roadmap and retrieve new roadmap index
    if (!hardware_interface_.WriteRoadmap(roadmap_spec.occupancy_file, roadmap_index))
    {
      ROS_ERROR_NAMED(LOGNAME, "Unable to write roadmap to hardware!");
      return false;
    }
    roadmap_indices_[roadmap_index] = roadmap;
  }
  return true;
}

void RTRPlannerInterface::processSolutionPath(const std::deque<unsigned int>& waypoints,
                                              const std::deque<unsigned int>& edges,
                                              robot_trajectory::RobotTrajectory& trajectory) const
{
}

}  // namespace rtr_interface
