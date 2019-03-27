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
 * Desc: Implementation of the RTRPlanningInterface
 */

// C++
#include <deque>
#include <string>
#include <vector>
#include <iostream>
#include <sstream>
#include <mutex>

// ROS
#include <ros/console.h>
#include <ros/console_backend.h>

// rtr_moveit
#include <rtr_moveit/rtr_planner_interface.h>
#include <rtr_moveit/roadmap_search.h>

namespace rtr_moveit
{
static const std::string LOGNAME = "rtr_planner_interface";

RTRPlannerInterface::RTRPlannerInterface(const ros::NodeHandle& nh) : nh_(nh)
{
  // Check if RapidPlan hardware should be used for collision checking
  rapidplan_interface_enabled_ = nh_.param("planner_config/rapidplan_interface_enabled", false);
  if (!rapidplan_interface_enabled_)
    ROS_WARN_NAMED(LOGNAME, "RapidPlanInterface is disabled - plans will be computed without collision checks");

  std::map<std::string, ros::console::levels::Level> loggers;
  if (ros::console::get_loggers(loggers))
  {
    auto logger_level = loggers.find(ros::this_node::getNamespace());
    if (logger_level != loggers.end())
      debug_ = logger_level->second == ros::console::levels::Level::Debug;
  }
}

RTRPlannerInterface::~RTRPlannerInterface()
{
  // TODO(RTR-52) implement destructor
}

bool RTRPlannerInterface::initialize()
{
  if (rapidplan_interface_enabled_)
  {
    // check if hardware is connected
    if (!rapidplan_interface_.Connected())
    {
      ROS_ERROR_NAMED(LOGNAME, "Unable to initialize RapidPlan interface. Hardware is not connected.");
      return false;
    }

    // try to initialize hardware
    if (!rapidplan_interface_.Init())
    {
      ROS_ERROR_NAMED(LOGNAME, "Unable to initialize RapidPlan interface. Failed to initialize Hardware.");
      return false;
    }

    // perform handshake
    if (!rapidplan_interface_.Handshake())
    {
      ROS_ERROR_NAMED(LOGNAME, "Unable to initialize RapidPlan interface. Handshake failed.");
      return false;
    }

    // clear hardware if  there are still roadmaps stored
    if (roadmap_indices_.empty())
    {
      size_t num_roadmaps;
      if (!rapidplan_interface_.NumRoadmaps(num_roadmaps))
      {
        ROS_ERROR_NAMED(LOGNAME, "Unable to initialize RapidPlan interface. Reading hardware state failed.");
        return false;
      }
      if (unsigned(num_roadmaps) > 0 && !rapidplan_interface_.ClearRoadmaps())
      {
        ROS_ERROR_NAMED(LOGNAME, "Unable to initialize RapidPlan interface. Clearing hardware storage failed.");
        return false;
      }
    }
  }

  ROS_INFO_NAMED(LOGNAME, "RapidPlan interface initialized.");
  return true;
}

bool RTRPlannerInterface::isReady() const
{
  // try handshake
  if (rapidplan_interface_enabled_ && !rapidplan_interface_.Handshake())
  {
    ROS_WARN_NAMED(LOGNAME, "RapidPlan interface is not ready. Handshake failed.");
    return false;
  }

  ROS_DEBUG_NAMED(LOGNAME, "RapidPlan interface is ready.");
  return true;
}

bool RTRPlannerInterface::solve(const RoadmapSpecification& roadmap_spec, const std::size_t start_state_id,
                                const RapidPlanGoal& goal, const OccupancyData& occupancy_data, const double& timeout,
                                std::vector<rtr::Config>& solution_path)
{
  std::deque<std::size_t> waypoints, edges;
  std::vector<rtr::Config> roadmap_states;
  bool success = solve(roadmap_spec, start_state_id, goal, occupancy_data, timeout, roadmap_states, waypoints, edges);
  // TODO(RTR-53): verify waypoints and states? This should already be done in the PathPlanner.
  if (success)
  {
    // fill solution path
    for (std::size_t waypoint : waypoints)
      solution_path.push_back(roadmap_states[waypoint]);

    // debug output
    if (debug_)
    {
      ROS_DEBUG_STREAM_NAMED(LOGNAME, "Solution path:");
      for (std::size_t waypoint : waypoints)
      {
        std::string waypoint_debug_text = "waypoint ";
        waypoint_debug_text += std::to_string(waypoint);
        waypoint_debug_text += ": ";
        for (float joint_value : roadmap_states[waypoint])
        {
          waypoint_debug_text += std::to_string(joint_value);
          waypoint_debug_text += " ";
        }
        ROS_DEBUG_STREAM_NAMED(LOGNAME, waypoint_debug_text);
      }
    }
  }
  return success;
}

bool RTRPlannerInterface::solve(const RoadmapSpecification& roadmap_spec, const std::size_t start_state_id,
                                const RapidPlanGoal& goal, const OccupancyData& occupancy_data, const double& timeout,
                                std::vector<rtr::Config>& roadmap_states, std::deque<std::size_t>& waypoints,
                                std::deque<std::size_t>& edges)
{
  {  // SCOPED MUTEX LOCK
    // In solve() the RapidPlanInterface and PathPlanner are loaded with the same roadmap so that results from
    // RapidPlanInterface::CheckScene() can be used with PathPlanner::FindPath().
    // Calling prepareRoadmap() ensures that both are loaded with the same roadmap and the mutex lock prevents race
    // conditions by restricting write access in the meantime.
    // All functions that either write or load roadmaps should follow this behavior.
    std::lock_guard<std::mutex> scoped_lock(mutex_);

    // Load roadmap to PathPlanner and MPA and get roadmap storage index
    size_t roadmap_index;
    if (!prepareRoadmap(roadmap_spec, roadmap_index))
      return false;

    // Check collisions using the RapidPlanInterface
    std::vector<uint8_t> collisions;
    if (rapidplan_interface_enabled_)
    {
      bool check_scene_success = false;
      if (occupancy_data.type == OccupancyData::Type::POINT_CLOUD)
        check_scene_success = rapidplan_interface_.CheckScene(occupancy_data.point_cloud, roadmap_index, collisions);
      else if (occupancy_data.type == OccupancyData::Type::VOXELS)
        check_scene_success = rapidplan_interface_.CheckScene(occupancy_data.voxels, roadmap_index, collisions);
      else
        ROS_WARN_NAMED(LOGNAME, "No type specified in occupancy data");

      if (!check_scene_success)
      {
        ROS_ERROR_NAMED(LOGNAME, "HardwareInterface failed to check collision scene.");
        return false;
      }
    }
    else
    {
      ROS_WARN_NAMED(LOGNAME, "RapidPlan called with disabled collision checks");
      collisions.resize(planner_.GetNumEdges());  // dummy
    }

    // Call PathPlanner
    int result = -1;
    if (goal.type == RapidPlanGoal::Type::TOOL_POSE)
    {
      result = planner_.FindPath(start_state_id, goal.tool_pose, collisions, goal.tolerance, goal.weights, waypoints,
                                 edges, timeout);
    }
    else if (goal.type == RapidPlanGoal::Type::STATE_IDS)
    {
      result = planner_.FindPath(start_state_id, goal.state_ids, collisions, waypoints, edges, timeout);
    }
    else
    {
      ROS_ERROR_NAMED(LOGNAME, "RapidPlanGoal goal type missing - Should be TOOL_POSE or STATE_IDS");
      return false;
    }

    // debug output
    if (debug_)
    {
      std::string waypoints_debug_text = "Waypoint ids: ";
      for (std::size_t waypoint : waypoints)
      {
        waypoints_debug_text += std::to_string(waypoint);
        waypoints_debug_text += " ";
      }
      ROS_DEBUG_STREAM_NAMED(LOGNAME, waypoints_debug_text);

      std::string edges_debug_text = "Edges: ";
      const std::vector<std::array<std::size_t, 2>>& roadmap_edges = planner_.GetEdges();
      for (std::size_t edge_id : edges)
      {
        edges_debug_text += std::to_string(roadmap_edges[(int)edge_id][0]);
        edges_debug_text += "-";
        edges_debug_text += std::to_string((int)roadmap_edges[(int)edge_id][1]);
        edges_debug_text += " ";
      }
      ROS_DEBUG_STREAM_NAMED(LOGNAME, edges_debug_text);
    }

    if (result != 0)  // FAILURE
    {
      ROS_ERROR_STREAM_NAMED(LOGNAME, "RapidPlan failed at finding a valid path - " << planner_.GetError(result));
      return false;
    }

    // SUCCESS
    ROS_INFO_STREAM_NAMED(LOGNAME, "RapidPlan found solution path with " << waypoints.size() << " waypoints");
    roadmap_states = planner_.GetConfigs();  // return state configs
    return result == 0;
  }  // SCOPED MUTEX UNLOCK
}

bool RTRPlannerInterface::getRoadmapConfigs(const RoadmapSpecification& roadmap_spec, std::vector<rtr::Config>& configs)
{
  // mutex locked because of sequential load and read access
  {  // SCOPED MUTEX LOCK
    std::lock_guard<std::mutex> scoped_lock(mutex_);
    bool success = loadRoadmapToPathPlanner(roadmap_spec);
    if (success)
      configs = planner_.GetConfigs();
    return success;
  }  // SCOPED MUTEX UNLOCK
}

bool RTRPlannerInterface::getRoadmapEdges(const RoadmapSpecification& roadmap_spec, std::vector<rtr::Edge>& edges)
{
  // mutex locked because of sequential load and read access
  {  // SCOPED MUTEX LOCK
    std::lock_guard<std::mutex> scoped_lock(mutex_);
    bool success = loadRoadmapToPathPlanner(roadmap_spec);
    if (success)
      edges = planner_.GetEdges();
    return success;
  }  // SCOPED MUTEX UNLOCK
}

bool RTRPlannerInterface::getRoadmapTransforms(const RoadmapSpecification& roadmap_spec,
                                               std::vector<rtr::ToolPose>& transforms)
{
  // mutex locked because of sequential load and read access
  {  // SCOPED MUTEX LOCK
    std::lock_guard<std::mutex> scoped_lock(mutex_);
    bool success = loadRoadmapToPathPlanner(roadmap_spec);
    if (success)
      transforms = planner_.GetTransforms();
    return success;
  }  // SCOPED MUTEX UNLOCK
}

bool RTRPlannerInterface::loadRoadmapToPathPlanner(const RoadmapSpecification& roadmap_spec)
{
  // check if roadmap is already loaded in the PathPlanner
  if (roadmap_spec.roadmap_id != loaded_roadmap_)
  {
    ROS_INFO_STREAM_NAMED(LOGNAME, "Loading roadmap: " << roadmap_spec.og_file);
    if (!planner_.LoadRoadmap(roadmap_spec.og_file))
    {
      ROS_ERROR_STREAM_NAMED(LOGNAME, "Failed to load roadmap '" << roadmap_spec.roadmap_id << "' to PathPlanner");
      std::cout << roadmap_spec.og_file << std::endl;
      return false;
    }

    // save new roadmap if it is new
    // TODO(RTR-51): Only store *.og file paths, others will be deprecated with the next API
    if (roadmaps_.find(roadmap_spec.roadmap_id) == roadmaps_.end())
      roadmaps_[roadmap_spec.roadmap_id] = roadmap_spec;
    loaded_roadmap_ = roadmap_spec.roadmap_id;

    // set edge cost as simple joint distance - TODO(RTR-55): use weighted distance?
    planner_.SetEdgeCost(&getConfigDistance);
  }
  return true;
}

bool RTRPlannerInterface::prepareRoadmap(const RoadmapSpecification& roadmap_spec, size_t& roadmap_index)
{
  if (!loadRoadmapToPathPlanner(roadmap_spec))
    return false;

  // check if roadmap is already written to hardware
  if (!findRoadmapIndex(roadmap_spec.roadmap_id, roadmap_index))
  {
    if (rapidplan_interface_enabled_)
    {
      // write roadmap and retrieve new roadmap index
      if (!rapidplan_interface_.WriteRoadmap(roadmap_spec.og_file, roadmap_index))
      {
        ROS_ERROR_STREAM_NAMED(LOGNAME, "Failed to write roadmap '" << roadmap_spec.roadmap_id << "' to RapidPlan MPA");
        return false;
      }
    }
    else
    {
      // if we don't use hardware, we just increase the numbers
      roadmap_index = roadmap_indices_.size();
    }
    roadmap_indices_[roadmap_index] = roadmap_spec.roadmap_id;
  }
  ROS_INFO_STREAM_NAMED(LOGNAME, "RapidPlan initialized with with roadmap '" << roadmap_spec.roadmap_id << "'");
  return true;
}
}  // namespace rtr_moveit
