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
 * Desc: Implementation of the RapidPlan planner plugin that can be loaded with the planning pipeline
 */

// C++
#include <map>
#include <set>
#include <string>
#include <vector>
#include <boost/filesystem.hpp>

// MoveIt!
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/Constraints.h>
#include <pluginlib/class_list_macros.hpp>

// rtr_moveit
#include <rtr_moveit/rtr_planning_context.h>
#include <rtr_moveit/rtr_planner_interface.h>
#include <rtr_moveit/roadmap_visualization.h>

// ROS parameter loading
#include <ros/package.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace rtr_moveit
{
const std::string LOGNAME = "rtr_planner_manager";

// planner interface constants
const std::string PLANNER_DESCRIPTION = "RapidPlan";
// default roadmap planner id (NOTE: in the future we could support diferent selection modes)
const std::string ROADMAP_DEFAULT = "Default";

class RTRPlannerManager : public planning_interface::PlannerManager
{
public:
  RTRPlannerManager() : planning_interface::PlannerManager(), nh_("~")
  {
  }

  /** \brief Initializes the planner interface and load parameters */
  bool initialize(const robot_model::RobotModelConstPtr& robot_model, const std::string& ns)
  {
    // load config
    group_names_ = robot_model->getJointModelGroupNames();
    loadRoadmapConfigurations();
    if (group_configs_.empty())
    {
      ROS_ERROR_NAMED(LOGNAME, "Failed at loading any group configurations from config file.");
      return false;
    }
    if (roadmaps_.empty())
    {
      ROS_ERROR_NAMED(LOGNAME, "Failed at loading any roadmap configurations from config file.");
      return false;
    }

    // initialize planner
    planner_interface_.reset(new RTRPlannerInterface(nh_));
    if (!planner_interface_->initialize())
    {
      ROS_ERROR_NAMED(LOGNAME, "RapidPlan interface could not be initialized!");
      return false;
    }

    // set default planner ids
    for (const std::pair<std::string, GroupConfig>& group_configs_item : group_configs_)
      nh_.setParam("/move_group/" + group_configs_item.first + "/default_planner_config", ROADMAP_DEFAULT);

    visualization_.reset(new RoadmapVisualization(nh_));

    return true;
  }

  /** \brief Verifies that the MotionPlanRequest can be handled by the planner */
  bool canServiceRequest(const moveit_msgs::MotionPlanRequest& req) const
  {
    // check if there is a roadmap configuration for the given group
    if (!group_configs_.count(req.group_name))
    {
      ROS_ERROR_STREAM_NAMED(LOGNAME, "No roadmap found for group " << req.group_name);
      return false;
    }

    // check if there are constraints that only contain joint/position/orientation
    bool has_valid_constraint = false;
    for (const moveit_msgs::Constraints& goal_constraint : req.goal_constraints)
    {
      if (goal_constraint.visibility_constraints.empty())
      {
        has_valid_constraint =
            !(goal_constraint.joint_constraints.empty() && goal_constraint.position_constraints.empty() &&
              goal_constraint.orientation_constraints.empty());
        if (has_valid_constraint)
          break;
      }
    }

    if (!has_valid_constraint)
    {
      ROS_ERROR_STREAM_NAMED(LOGNAME, "Planning request does not contain supported goal constraints - "
                                          << "Supported are only joint/position/orientation goals");
      return false;
    }
    // Note: Further checks require inspecting the roadmaps, volume region and attached collision objects.
    // This does not belong into the planner manager and should be provided by the planning context.
    // TODO(RTR-50): check if we have a valid roadmap for this request
    // TODO(RTR-49): check if we can handle the goal constraints

    return true;
  }

  /** \brief Returns the planner description */
  std::string getDescription() const
  {
    return PLANNER_DESCRIPTION;
  }

  /** \brief Returns the names of available planning algorithms */
  void getPlanningAlgorithms(std::vector<std::string>& algs) const
  {
    algs.clear();
    for (const std::string& group_name : group_names_)
      algs.push_back(group_name);
    for (const std::pair<std::string, GroupConfig>& group_configs_item : group_configs_)
    {
      algs.push_back(group_configs_item.first + "[" + ROADMAP_DEFAULT + "]");
      for (const std::string& roadmap_id : group_configs_item.second.roadmap_ids)
        algs.push_back(group_configs_item.first + "[" + roadmap_id + "]");
    }
  }

  void loadRoadmapConfigurations()
  {
    // load group configs
    group_configs_.clear();
    std::set<std::string> roadmap_ids;
    for (const std::string& group_name : group_names_)
    {
      if (!nh_.hasParam(group_name))
      {
        ROS_INFO_STREAM_NAMED(LOGNAME, "No roadmap specification found for group " << group_name);
        continue;
      }

      // create new group config
      GroupConfig config;
      config.group_name = group_name;
      std::vector<std::string> group_roadmap_ids;
      rosparam_shortcuts::get(LOGNAME, nh_, group_name + "/default_roadmap", config.default_roadmap_id);
      rosparam_shortcuts::get(LOGNAME, nh_, group_name + "/roadmaps", group_roadmap_ids);

      // check if default roadmap is set
      if (!config.default_roadmap_id.empty())
        config.roadmap_ids.insert(config.default_roadmap_id);
      else
        ROS_WARN_STREAM("No default roadmap specified for group " << group_name);

      // add specified roadmap names
      config.roadmap_ids.insert(group_roadmap_ids.begin(), group_roadmap_ids.end());

      // leave out group if no roadmap was found
      if (config.roadmap_ids.empty())
      {
        ROS_INFO_STREAM("Leaving out group " << group_name << ", no roadmaps are specified in the config file.");
        continue;
      }

      // add new roadmap ids
      roadmap_ids.insert(config.roadmap_ids.begin(), config.roadmap_ids.end());

      // add group config
      group_configs_[group_name] = config;
    }

    // load default configs
    std::string default_roadmaps_package;
    std::string default_roadmaps_directory;
    rosparam_shortcuts::get(LOGNAME, nh_, "default/roadmaps_package", default_roadmaps_package);
    rosparam_shortcuts::get(LOGNAME, nh_, "default/roadmaps_directory", default_roadmaps_directory);
    if (ros::package::getPath(default_roadmaps_package).empty())
      ROS_WARN_STREAM_NAMED(LOGNAME, "Unable to find default roadmaps package '" << default_roadmaps_package << "'");

    // load roadmap specifications
    roadmaps_.clear();
    for (const std::string& roadmap_id : roadmap_ids)
    {
      // look for non-default parameters
      std::string roadmap_config = "roadmaps/" + roadmap_id;
      std::string package = nh_.param(roadmap_config + "/package", default_roadmaps_package);
      std::string directory = nh_.param(roadmap_config + "/directory", default_roadmaps_directory);
      std::string filename = nh_.param(roadmap_config + "/filename", roadmap_id);

      // skip roadmap if package is invalid
      std::string package_path = ros::package::getPath(package);
      if (package_path.empty())
      {
        std::string skip_roadmap_message = "Skipping roadmap '" + roadmap_id + "': ";
        if (package != default_roadmaps_package)
          ROS_WARN_STREAM_NAMED(LOGNAME, skip_roadmap_message << "invalid package '" << package << "'");
        else
          ROS_WARN_STREAM_NAMED(LOGNAME, skip_roadmap_message << "invalid default package");
        continue;
      }

      // resolve roadmap file
      boost::filesystem::path roadmap_file(package_path);
      roadmap_file.append(directory);
      roadmap_file.append(filename);
      if (!roadmap_file.has_extension())
        roadmap_file.replace_extension(".og");

      // check if file exists
      if (!boost::filesystem::exists(roadmap_file))
      {
        ROS_WARN_STREAM_NAMED(LOGNAME, "Unable to locate file for roadmap '" << roadmap_id << "' at: " << roadmap_file);
        continue;
      }

      // add new roadmap spec
      RoadmapSpecification spec;
      spec.roadmap_id = roadmap_id;
      spec.og_file = roadmap_file.c_str();
      roadmaps_[roadmap_id] = spec;
      ROS_INFO_STREAM_NAMED(LOGNAME, "Found roadmap '" << roadmap_id << "' at: " << roadmap_file);
    }
  }

  /** \brief Returns a configured planning context for a given planning scene and motion plan request */
  planning_interface::PlanningContextPtr getPlanningContext(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                            const planning_interface::MotionPlanRequest& req,
                                                            moveit_msgs::MoveItErrorCodes& error_code) const
  {
    RTRPlanningContextPtr context;
    error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
    // look for group config
    auto group_config_search = group_configs_.find(req.group_name);
    if (group_config_search != group_configs_.end())
    {
      // look for roadmap
      // TODO(RTR-31): look for suitable roadmap if we have multiple
      GroupConfig group_config = group_config_search->second;
      std::string group_roadmap = group_config.default_roadmap_id;
      // look for non-default roadmap id
      if (req.planner_id != ROADMAP_DEFAULT && req.planner_id != group_roadmap)
      {
        auto roadmap_search = group_config.roadmap_ids.find(req.planner_id);
        if (roadmap_search != group_config.roadmap_ids.end())
          group_roadmap = *roadmap_search;
      }
      // if default roadmap is not specified, use the first in the list
      else if (group_roadmap.empty() && !group_config.roadmap_ids.empty())
      {
        group_roadmap = *group_config.roadmap_ids.begin();
      }
      auto roadmap_search = roadmaps_.find(group_roadmap);
      if (roadmap_search != roadmaps_.end())
      {
        context.reset(
            new RTRPlanningContext(req.group_name, roadmap_search->second, planner_interface_, visualization_));
        context->setMotionPlanRequest(req);
        context->setPlanningScene(planning_scene);
        context->configure(error_code);
      }
      else
      {
        ROS_ERROR_STREAM_NAMED(LOGNAME, "No valid roadmap specification found for group " << req.group_name);
      }
    }
    else
    {
      ROS_ERROR_STREAM_NAMED(LOGNAME, "Group '" << req.group_name << "' is not configured with any roadmaps for "
                                                                     "RapidPlan");
    }
    return context;
  }

private:
  ros::NodeHandle nh_;

  // The RapidPlan wrapper interface
  RTRPlannerInterfacePtr planner_interface_;
  RoadmapVisualizationPtr visualization_;

  // group and roadmap configurations
  std::vector<std::string> group_names_;
  std::map<std::string, GroupConfig> group_configs_;
  std::map<std::string, RoadmapSpecification> roadmaps_;
};
}  // namespace rtr_moveit

PLUGINLIB_EXPORT_CLASS(rtr_moveit::RTRPlannerManager, planning_interface::PlannerManager);
