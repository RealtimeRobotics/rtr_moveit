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
 * Desc: Implementation of the RTRPlanningContext
 */

// C++
#include <string>
#include <vector>
#include <algorithm>

// Eigen
#include <Eigen/Geometry>

// ROS parameters
#include <ros/ros.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <tf/transform_datatypes.h>

// MoveIt! constraints
#include <moveit/constraint_samplers/constraint_sampler.h>
#include <moveit/constraint_samplers/default_constraint_samplers.h>
#include <moveit/constraint_samplers/union_constraint_sampler.h>
#include <moveit_msgs/Constraints.h>
#include <moveit/robot_state/conversions.h>

// rtr_moveit
#include <rtr_moveit/rtr_planning_context.h>
#include <rtr_moveit/rtr_planner_interface.h>
#include <rtr_moveit/occupancy_handler.h>
#include <rtr_moveit/roadmap_search.h>

// RapidPlan
#include <rtr-api/OGFileReader.hpp>

namespace rtr_moveit
{
static const std::string LOGNAME = "rtr_planning_context";
RTRPlanningContext::RTRPlanningContext(const std::string& planning_group, const RoadmapSpecification& roadmap_spec,
                                       const RTRPlannerInterfacePtr& planner_interface)
  : planning_interface::PlanningContext(planning_group + "[" + roadmap_spec.roadmap_id + "]", planning_group),
  planner_interface_(planner_interface),
  roadmap_(roadmap_spec)
{
}

moveit_msgs::MoveItErrorCodes RTRPlanningContext::solve(robot_trajectory::RobotTrajectoryPtr& trajectory,
                                                        double& planning_time)
{
  ros::Time start_time = ros::Time::now();
  terminate_plan_time_ = start_time + ros::Duration(request_.allowed_planning_time);
  moveit_msgs::MoveItErrorCodes result;
  result.val = result.FAILURE;

  // this should always be satisfied since getPlanningContext() would have failed otherwise
  if (!configured_)
  {
    ROS_ERROR_NAMED(LOGNAME, "solve() was called but planning context has not been configured successfully");
    return result;
  }

  // extract RapidPlanGoals;
  if (!initRapidPlanGoals(request_.goal_constraints, goals_))
    return result;

  // prepare collision scene
  bool occupancy_success;
  OccupancyData occupancy_data;
  OccupancyHandler occupancy_handler;
  occupancy_handler.setVisualizationEnabled(visualization_enabled_);
  occupancy_handler.setVolumeRegion(roadmap_.volume);
  if (occupancy_source_ == "POINT_CLOUD")
    occupancy_success = occupancy_handler.fromPointCloud(pcl_topic_, occupancy_data);
  else
    occupancy_success = occupancy_handler.fromPlanningScene(planning_scene_, occupancy_data);
  if (!occupancy_success)
    return result;

  // initialize start state
  unsigned int start_state_id;
  if (!initStartState(start_state_id))
    return result;

  // Iterate goals and plan until we have a solution
  std::vector<rtr::Config> solution_path;
  result.val = result.PLANNING_FAILED;
  for (std::size_t goal_pos = 0; goal_pos < goals_.size(); goal_pos++)
  {
    // check time
    double timeout = (terminate_plan_time_ - ros::Time::now()).toSec() * 1000;  // seconds -> milliseconds
    if (timeout <= 0.0)
    {
      result.val = moveit_msgs::MoveItErrorCodes::TIMED_OUT;
      break;
    }

    // run plan
    const RapidPlanGoal& goal = goals_[goal_pos];
    if (planner_interface_->solve(roadmap_, start_state_id, goal, occupancy_data, timeout, solution_path))
    {
      if (solution_path.empty())
      {
        ROS_WARN_NAMED(LOGNAME, "Cannot convert empty path to robot trajectory");
        continue;
      }

      // convert solution path to robot trajectory
      const robot_state::RobotState& reference_state = planning_scene_->getCurrentState();
      trajectory.reset(new robot_trajectory::RobotTrajectory(reference_state.getRobotModel(), group_));
      processSolutionPath(solution_path, reference_state, joint_model_names_, *trajectory);

      // connect start state waypoint
      if (!connectWaypointToTrajectory(trajectory, start_state_, true))
      {
        ROS_WARN_NAMED(LOGNAME, "Found collisions trying to connect the requested start state to the solution path");
        continue;
      }

      // connect goal state waypoint
      if (goal_states_[goal_pos])
      {
        if (!connectWaypointToTrajectory(trajectory, goal_states_[goal_pos]))
        {
          ROS_WARN_NAMED(LOGNAME, "Found collisions trying to connect the goal state to the solution path");
          continue;
        }
      }

      // plan successful
      result.val = result.SUCCESS;
      break;
    }
    solution_path.clear();
  }
  planning_time = (ros::Time::now() - start_time).toSec();
  return result;
}

bool RTRPlanningContext::solve(planning_interface::MotionPlanResponse& res)
{
  res.error_code_ = solve(res.trajectory_, res.planning_time_);
  return res.error_code_.val == res.error_code_.SUCCESS;
}

bool RTRPlanningContext::solve(planning_interface::MotionPlanDetailedResponse& res)
{
  res.trajectory_.resize(res.trajectory_.size() + 1);
  res.processing_time_.resize(res.processing_time_.size() + 1);
  res.error_code_ = solve(res.trajectory_.back(), res.processing_time_.back());
  res.description_.push_back("plan");
  //TODO(henningkayser): add more detailed descriptions for planning steps
  return res.error_code_.val == res.error_code_.SUCCESS;
}

void RTRPlanningContext::processSolutionPath(const std::vector<rtr::Config>& solution_path,
                                             const robot_state::RobotState& reference_state,
                                             const std::vector<std::string>& joint_names,
                                             robot_trajectory::RobotTrajectory& trajectory)
{
  ROS_ASSERT_MSG(joint_names.size() == solution_path[0].size(), "Joint values don't match joint names");
  for (const rtr::Config& joint_config : solution_path)
  {
    robot_state::RobotStatePtr robot_state(new robot_state::RobotState(reference_state));
    for (std::size_t i = 0; i < joint_names.size(); i++)
      robot_state->setJointPositions(joint_names[i], { (double)joint_config[i] });
    trajectory.addSuffixWayPoint(robot_state, 0.1);
  }
}

bool RTRPlanningContext::connectWaypointToTrajectory(const robot_trajectory::RobotTrajectoryPtr& trajectory,
                                                     const robot_state::RobotStatePtr& waypoint_state,
                                                     bool connect_to_front)
{
  robot_state::RobotState connecting_state(trajectory->getLastWayPoint());
  if (connect_to_front)
    connecting_state = trajectory->getFirstWayPoint();

  // check collisions of intermediate states and the waypoint state itself
  robot_state::RobotState intermediate_state(connecting_state);
  double waypoint_distance = connecting_state.distance(*waypoint_state);
  std::size_t step_count = std::abs(waypoint_distance / max_waypoint_distance_) + 1;
  double step_fraction = 1.0 / step_count;
  for (std::size_t step = 0; step <= step_count; step++)
  {
    connecting_state.interpolate(*waypoint_state, step * step_fraction, intermediate_state);
    if (planning_scene_->isStateColliding(intermediate_state))
      return false;
  }
  if (connect_to_front)
    trajectory->addPrefixWayPoint(intermediate_state, 0.0);
  else
    trajectory->addSuffixWayPoint(intermediate_state, 0.0);
  return true;
}

void RTRPlanningContext::configure(moveit_msgs::MoveItErrorCodes& error_code)
{
  error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;

  // load defult planner parameters
  // TODO(RTR-56): support overloading defaults
  ros::NodeHandle nh("~");
  std::size_t error = 0;
  error += !rosparam_shortcuts::get(LOGNAME, nh, "planner_config/allowed_position_distance", allowed_position_distance_);
  error += !rosparam_shortcuts::get(LOGNAME, nh, "planner_config/allowed_joint_distance", allowed_joint_distance_);
  error += !rosparam_shortcuts::get(LOGNAME, nh, "planner_config/max_goal_states", max_goal_states_);
  error += !rosparam_shortcuts::get(LOGNAME, nh, "planner_config/max_waypoint_distance", max_waypoint_distance_);
  if (error)
  {
    ROS_ERROR_NAMED(LOGNAME, "Planning Context could not be configured due to missing params");
    return;
  }

  // read occupancy parameters
  nh.param("planner_config/occupancy_source", occupancy_source_, std::string("PLANNING_SCENE"));
  nh.param("planner_config/visualization_enabled", visualization_enabled_, false);
  if (occupancy_source_ != "PLANNING_SCENE")
  {
    if (occupancy_source_ != "POINT_CLOUD")
    {
      ROS_WARN_STREAM_NAMED(LOGNAME, "Occupancy source is set to unknown type '" << occupancy_source_
                                  << "'. Proceeding with default 'PLANNING_SCENE'.");
      occupancy_source_ = "PLANNING_SCENE";
    }
    else if (!nh.getParam("planner_config/pcl_topic", pcl_topic_))
    {
      ROS_ERROR_NAMED(LOGNAME, "Occupancy source 'POINT_CLOUD' cannot be configured without parameter 'pcl_topic'");
      return;
    }
  }


  // planning scene should be set
  if (!planning_scene_)
  {
    ROS_ERROR_NAMED(LOGNAME, "Cannot configure planning context while planning scene has not been set");
    return;
  }

  // get joint model group
  jmg_ = planning_scene_->getCurrentState().getJointModelGroup(group_);
  joint_model_names_ = jmg_->getActiveJointModelNames();

  // check planner interface
  if (!planner_interface_->isReady() && !planner_interface_->initialize())
    return;

  // read roadmap data from .og file
  og_file_.reset(new rtr::OGFileReader(roadmap_.og_file));
  if (!og_file_->IsValid())
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Roadmap file invalid " << roadmap_.og_file << "'");
    return;
  }

  // get roadmap configs
  if (!og_file_->GetConfigs(roadmap_configs_) || roadmap_configs_.empty())
  {
    ROS_ERROR_NAMED(LOGNAME, "Unable to load config states from roadmap file");
    return;
  }

  // check if joint dimension in roadmap fits to joint model group
  if (roadmap_configs_[0].size() != joint_model_names_.size())
  {
    ROS_ERROR_NAMED(LOGNAME, "Roadmap state dimension does not fit to joint count of planning group");
    return;
  }

  // get roadmap poses
  if (!og_file_->GetPoses(roadmap_poses_) || roadmap_poses_.empty())
  {
    ROS_ERROR_NAMED(LOGNAME, "Unable to load state poses from roadmap file");
    return;
  }

  // load occupancy region volume
  rtr::ToolPose volume_center_pose;
  if (!og_file_->GetVoxelRegion(roadmap_.volume.pose.header.frame_id, volume_center_pose, roadmap_.volume.dimension))
  {
    ROS_ERROR_NAMED(LOGNAME, "Unable to load voxel region from roadmap file");
    return;
  }
  roadmap_.volume.pose.pose.position.x = volume_center_pose[0];
  roadmap_.volume.pose.pose.position.y = volume_center_pose[1];
  roadmap_.volume.pose.pose.position.z = volume_center_pose[2];
  roadmap_.volume.pose.pose.orientation =
    tf::createQuaternionMsgFromRollPitchYaw(volume_center_pose[3], volume_center_pose[4], volume_center_pose[5]);
  roadmap_.volume.pose.header.frame_id = "world"; // NOTE: GetVoxelRegion returns an empty frame - we fix this here
  roadmap_.volume.voxel_resolution = {64, 64, 64}; // NOTE: these are the hardcoded/fixed values from the rtr-toolkit

  // done
  error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  configured_ = true;
}

bool RTRPlanningContext::initRapidPlanGoals(const std::vector<moveit_msgs::Constraints>& goal_constraints,
                                           std::vector<RapidPlanGoal>& goals)
{
  bool success = false;
  for (const moveit_msgs::Constraints& goal_constraint : goal_constraints)
  {
    RapidPlanGoal goal;
    robot_state::RobotStatePtr goal_state;
    if (getRapidPlanGoal(goal_constraint, goal, goal_state))
    {
      goals.push_back(goal);
      goal_states_.push_back(goal_state);
    }
  }
  std::string error_msg;
  if (goal_constraints.empty())
    ROS_ERROR_NAMED(LOGNAME, "Goal constraints are empty");
  else if (goals.empty())
    ROS_ERROR_NAMED(LOGNAME, "Failed to extract any goals from constraints");
  else
    success = true;
  return success;
}

bool RTRPlanningContext::getRapidPlanGoal(const moveit_msgs::Constraints& goal_constraint, RapidPlanGoal& goal,
                                          robot_state::RobotStatePtr& goal_state)
{
  goal.type = RapidPlanGoal::Type::STATE_IDS;

  // initialize constraint samplers
  std::vector<constraint_samplers::ConstraintSamplerPtr> samplers;
  // joint constraint
  if (!goal_constraint.joint_constraints.empty())
  {
    // joint state sampler
    constraint_samplers::JointConstraintSamplerPtr joint_sampler(
        new constraint_samplers::JointConstraintSampler(planning_scene_, group_));
    joint_sampler->configure(goal_constraint);
    samplers.push_back(joint_sampler);
  }
  // position/orientation constraint
  if (!goal_constraint.position_constraints.empty() || !goal_constraint.orientation_constraints.empty())
  {
    // IK sampler
    constraint_samplers::IKConstraintSamplerPtr ik_sampler(
        new constraint_samplers::IKConstraintSampler(planning_scene_, group_));
    ik_sampler->configure(goal_constraint);
    samplers.push_back(ik_sampler);
  }

  // sample goal from roadmap states
  constraint_samplers::UnionConstraintSampler union_sampler(planning_scene_, group_, samplers);
  const robot_state::RobotState& robot_state = planning_scene_->getCurrentState();
  robot_state::RobotState sample_state(robot_state);
  std::vector<double> joint_positions(jmg_->getActiveJointModels().size());
  rtr::Config sample_config(joint_positions.size());
  std::vector<float> distances;
  while (ros::Time::now() < terminate_plan_time_)
  {
    if (!union_sampler.sample(sample_state, robot_state, 100))
      continue;
    sample_state.copyJointGroupPositions(group_, joint_positions);
    // copy joint values to rtr::Config
    std::transform(std::begin(joint_positions), std::end(joint_positions), std::begin(sample_config),
                   [](double d) -> float {return float(d);});
    // search for goal state candidates within allowed joint distance
    //TODO(RTR-7): (pre-)filter by allowed position distance
    findClosestConfigs(sample_config, roadmap_configs_, goal.state_ids, distances, max_goal_states_,
                       allowed_joint_distance_);
    if (!goal.state_ids.empty())
    {
      goal_state = std::make_shared<robot_state::RobotState>(sample_state);
      return true;
    }
  }
  return false;
}

bool RTRPlanningContext::initStartState(unsigned int& start_state_id)
{
  rtr::Config start_config;
  start_state_ = std::make_shared<robot_state::RobotState>(planning_scene_->getCurrentState());

  // convert start state from MotionPlanRequest
  if (!request_.start_state.joint_state.position.empty())
  {
    std::size_t num_joints = request_.start_state.joint_state.position.size();
    for (const std::string& joint_name : joint_model_names_)
      for (std::size_t i = 0; i < num_joints; ++i)
        if (joint_name == request_.start_state.joint_state.name[i])
          start_config.push_back(request_.start_state.joint_state.position[i]);
    if (start_config.size() != joint_model_names_.size())
    {
      ROS_ERROR_NAMED(LOGNAME, "Invalid start state in planning request - joint message does not match to joint group");
      return false;
    }
    // write requested joint values to start state
    start_state_->setVariablePositions(request_.start_state.joint_state.name,
                                       request_.start_state.joint_state.position);
  }
  else
  {
    // if start state in request is not populated, take the current state of the planning scene
    ROS_WARN_NAMED(LOGNAME, "Start state in MotionPlanRquest is not populated - using current state from planning scene.");
    std::vector<double> joint_positions;
    start_state_->copyJointGroupPositions(group_, joint_positions);
    for (const double& joint_position : joint_positions)
      start_config.push_back(joint_position);
  }

  // search for start state candidate in roadmap
  int result_id = findClosestConfigId(start_config, roadmap_configs_, allowed_joint_distance_);
  if (result_id < 0)
    ROS_ERROR_NAMED(LOGNAME, "Unable to find a start state candidate in the roadmap within the allowed joint distance");
  start_state_id = result_id;
  return result_id >= 0;
}

void RTRPlanningContext::clear()
{
  //TODO(henningkayser): implement and support reusing planning contexts
}

bool RTRPlanningContext::terminate()
{
  // RapidPlan does not support this right now
  ROS_WARN_STREAM_NAMED(LOGNAME, "Failed to terminate the planning attempt! This is not supported.");
  return false;
}
}  // namespace rtr_moveit
