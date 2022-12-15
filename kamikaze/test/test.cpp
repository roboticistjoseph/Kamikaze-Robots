/**
 * @file test.cpp
 * @author Aneesh Chodisetty (aneeshc@umd.edu)
 * @author Bhargav Kumar Soothram (bsoothra@umd.edu)
 * @author Joseph Pranadheer Reddy Katakam (jkatak@umd.edu)
 * @brief 
 * @version 0.1
 * @date 2022-12-15
 * 
 * @copyright 
 *  // Copyright 2016 Open Source Robotics Foundation, Inc.
    //
    // Licensed under the Apache License, Version 2.0 (the "License");
    // you may not use this file except in compliance with the License.
    // You may obtain a copy of the License at
    //
    //     http://www.apache.org/licenses/LICENSE-2.0
    //
    // Unless required by applicable law or agreed to in writing, software
    // distributed under the License is distributed on an "AS IS" BASIS,
    // WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    // See the License for the specific language governing permissions and
    // limitations under the License.
  */

// include standard libraries
#include <stdlib.h>

// Google test library for Unit Testing
#include <gtest/gtest.h>

// include ros libraries
// #include <ros/ros.h>
// #include <geometry_msgs/Twist.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

/**
 * @brief Test to check if velocity commands are being published correctly
 * 
 */
// TEST(BotControlTest, checkVelocityCommands) {
//   ros::NodeHandle nh;
//   ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

//   // Send a velocity command to the Turtlebot
//   geometry_msgs::Twist cmd;
//   cmd.linear.x = 0.1;
//   cmd.angular.z = 0.2;
//   cmd_vel_pub.publish(cmd);

//   // Wait for the command to be received and processed
//   ros::Duration(1.0).sleep();

//   // Check if the Turtlebot is moving with the expected velocity
//   geometry_msgs::Twist current_vel = getCurrentVelocity();
//   EXPECT_FLOAT_EQ(current_vel.linear.x, cmd.linear.x);
//   EXPECT_FLOAT_EQ(current_vel.angular.z, cmd.angular.z);
// }

/**
 * @brief Dummy Test
 * 
 */
TEST(dummyTest, checkVelocityCommands) {
  EXPECT_EQ(1, 1);
}

/**
 * @brief Main file to run the Test Cases
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();
  std::cout << "DONE SHUTTING DOWN ROS" << std::endl;
  return result;
}
