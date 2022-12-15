/**
 * @file spawn_controller.h
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

// preprocessor directives
#ifndef \
  ROS2_WS2_SRC_KAMIKAZE_ROBOTS_APP_KAMIKAZE_INCLUDE_KAMIKAZE_SPAWN_CONTROLLER_H_
#define \
  ROS2_WS2_SRC_KAMIKAZE_ROBOTS_APP_KAMIKAZE_INCLUDE_KAMIKAZE_SPAWN_CONTROLLER_H_

// include standard libraries
#include <string>
#include <memory>

// include ros libraries
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

/**
 * @brief Class file for managing the formation of Bots in Gazebo environment
 *
 */
class SpawnController : public rclcpp::Node {
public:

  /**
   * @brief Construct a new Spawn Controller object
   *
   * @param node_name
   */
  explicit SpawnController(const std::string& node_name = "spawn_controller");

private:

  /**
   * @brief Publisher for each Bot
   *
   */
  std::array<rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr,
             20>publishers_;

  //   rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;  //!<
  // The pointer to the publisher.

  /**
   * @brief The calculated bot velocity.
   *
   */
  geometry_msgs::msg::Twist bot_velocity_;

  /**
   * @brief The pointer to the publisher callback.
   *
   */
  rclcpp::TimerBase::SharedPtr timer_publisher_;

  /**
   * @brief Counter variable for callback function
   *
   */
  int callback_counter_;

  /**
   * @brief The callback funciton
   */
  void timer_callback();
};

#endif // ROS2_WS2_SRC_KAMIKAZE_ROBOTS_APP_KAMIKAZE_INCLUDE_KAMIKAZE_SPAWN_CONTROLLER_H_
