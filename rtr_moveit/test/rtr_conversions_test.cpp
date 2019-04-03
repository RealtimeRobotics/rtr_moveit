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
 * Desc: henningkayser@picknik.ai
 */

// C++
#include <limits>
#include <vector>
#include <string>

// gtest
#include <gtest/gtest.h>

// package dependencies
#include <rtr_moveit/occupancy_handler.h>
#include <rtr_moveit/rtr_datatypes.h>

// planning scene conversion
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit_msgs/CollisionObject.h>
#include <srdfdom/model.h>
#include <urdf_model/model.h>

// RapidPlan
#include <rtr-occupancy/Voxel.hpp>

/* This test creates an empy planning scene and checks the number of occupancy
 * voxels after conversion for different configurations. */
// NOTE: The test throws warnings that frame transforms are empty and the
// planning scene falls back to 'identity' This is ok and does
// not impair the results.
TEST(TestSuite, convertPlanningScene)
{
  ros::NodeHandle nh;

  // instantiate emtpy planning scene
  urdf::ModelInterfaceSharedPtr urdf_model;
  urdf_model.reset(new urdf::ModelInterface());
  srdf::ModelConstSharedPtr srdf_model;
  srdf_model.reset(new srdf::Model());
  moveit::core::RobotModelConstPtr robot_model;
  robot_model.reset(new moveit::core::RobotModel(urdf_model, srdf_model));
  planning_scene::PlanningScenePtr scene;
  scene.reset(new planning_scene::PlanningScene(robot_model));

  // specify volume region
  rtr_moveit::RoadmapVolume volume;
  volume.pose.header.frame_id = scene->getPlanningFrame();
  volume.pose.pose.orientation.w = 1.0;
  volume.dimension[0] = 1.0;
  volume.dimension[1] = 1.0;
  volume.dimension[2] = 1.0;
  volume.voxel_resolution[0] = 10;
  volume.voxel_resolution[1] = 10;
  volume.voxel_resolution[2] = 10;

  // convert planning scene object to voxels
  rtr_moveit::OccupancyHandler occupancy_handler(nh);
  occupancy_handler.setVolumeRegion(volume);
  rtr_moveit::OccupancyData occupancy;
  occupancy_handler.fromPlanningScene(scene, occupancy);

  // voxels should be empty
  EXPECT_TRUE(occupancy.voxels.empty()) << "Created " << occupancy.voxels.size() << " occupancy voxels for empty "
                                                                                    "planning scene!";

  // create collision object
  moveit_msgs::CollisionObject obj;
  obj.id = "collision_object";
  obj.header.frame_id = scene->getPlanningFrame();
  obj.primitives.resize(1);
  obj.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  obj.primitives[0].dimensions.resize(3);
  obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 1.0;
  obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 1.0;
  obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 1.0;
  obj.primitive_poses.resize(1);
  obj.primitive_poses[0].orientation.w = 1.0;
  obj.primitive_poses[0].position.x += 0.501 * volume.dimension[shape_msgs::SolidPrimitive::BOX_X];
  obj.primitive_poses[0].position.y += 0.501 * volume.dimension[shape_msgs::SolidPrimitive::BOX_Y];
  obj.primitive_poses[0].position.z += 0.501 * volume.dimension[shape_msgs::SolidPrimitive::BOX_Z];
  obj.operation = moveit_msgs::CollisionObject::ADD;

  // add object to planning scene
  scene->processCollisionObjectMsg(obj);

  // There should be 1000 occupancy voxels (of 1000)
  occupancy.voxels.clear();
  occupancy_handler.fromPlanningScene(scene, occupancy);
  EXPECT_TRUE(occupancy.voxels.size() == 1000) << "Created " << occupancy.voxels.size()
                                               << " occupancy voxels even though there should be 1000.";

  // shift volume so only half of it is occluded
  volume.pose.pose.position.x += 0.501 * volume.dimension[shape_msgs::SolidPrimitive::BOX_X];
  occupancy_handler.setVolumeRegion(volume);
  occupancy.voxels.clear();
  occupancy_handler.fromPlanningScene(scene, occupancy);
  EXPECT_TRUE(occupancy.voxels.size() == 500) << "Created " << occupancy.voxels.size() << " occupancy voxels even "
                                                                                          "though there should be 500";

  // shift volume so only a quarter of it is occluded
  volume.pose.pose.position.y += 0.501 * volume.dimension[shape_msgs::SolidPrimitive::BOX_Y];
  occupancy_handler.setVolumeRegion(volume);
  occupancy.voxels.clear();
  occupancy_handler.fromPlanningScene(scene, occupancy);
  EXPECT_TRUE(occupancy.voxels.size() == 250) << "Created " << occupancy.voxels.size() << " occupancy voxels even "
                                                                                          "though there should be 250";

  // shift volume so only an eight of it is occluded
  volume.pose.pose.position.z += 0.501 * volume.dimension[shape_msgs::SolidPrimitive::BOX_Z];
  occupancy_handler.setVolumeRegion(volume);
  occupancy.voxels.clear();
  occupancy_handler.fromPlanningScene(scene, occupancy);
  EXPECT_TRUE(occupancy.voxels.size() == 125) << "Created " << occupancy.voxels.size() << " occupancy voxels even "
                                                                                          "though there should be 125";
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "rtr_conversions_test");
  return RUN_ALL_TESTS();
}
