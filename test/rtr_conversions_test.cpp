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

#include <limits>
#include <gtest/gtest.h>
#include <rtr_interface/rtr_conversions.h>

TEST(TestSuite, convertPoseAndTransform)
{
  // create pose and test pose
  geometry_msgs::Pose pose, test_pose;
  pose.position.x = 0.5;
  pose.position.y = 0.3;
  pose.position.z = 0.9;
  tf::Quaternion rotation = tf::createQuaternionFromRPY(0.1, 0.4, 0.2);
  tf::quaternionTFToMsg(rotation, pose.orientation);

  std::array<float, 6> transform, test_transform;

  // convert pose to transform and back
  rtr_interface::poseMsgToRTR(pose, transform);
  rtr_interface::poseRTRToMsg(transform, test_pose);
  rtr_interface::poseMsgToRTR(test_pose, test_transform);

  // equality threshold
  double threshold = std::numeric_limits<float>::epsilon();

  // test pose equality
  EXPECT_TRUE(std::abs(pose.position.x - test_pose.position.x) < threshold);
  EXPECT_TRUE(std::abs(pose.position.y - test_pose.position.y) < threshold);
  EXPECT_TRUE(std::abs(pose.position.z - test_pose.position.z) < threshold);

  // test rotation equality
  tf::Quaternion test_rotation;
  tf::quaternionMsgToTF(test_pose.orientation, test_rotation);
  EXPECT_TRUE(std::abs(rotation.angleShortestPath(test_rotation)) < 0.1) << "Pose: Angle shortest path "
                                                                         << rotation.angleShortestPath(test_rotation);

  // test transform equality from the other side
  EXPECT_TRUE(std::abs(transform[0] - test_transform[0]) < threshold);
  EXPECT_TRUE(std::abs(transform[1] - test_transform[1]) < threshold);
  EXPECT_TRUE(std::abs(transform[2] - test_transform[2]) < threshold);
  rotation = tf::createQuaternionFromRPY(transform[0], transform[1], transform[2]);
  test_rotation = tf::createQuaternionFromRPY(test_transform[0], test_transform[1], test_transform[2]);
  EXPECT_TRUE(std::abs(rotation.angleShortestPath(test_rotation)) < 0.1) << "Transform: Angle shortest path "
                                                                         << rotation.angleShortestPath(test_rotation);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  // ros::init(argc, argv, "tester");
  // ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
