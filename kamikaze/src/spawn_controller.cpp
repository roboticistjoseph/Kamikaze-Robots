/**
 * @file spawn_controller.cpp
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

// include header file
#include <spawn_controller.hpp>

SpawnController::SpawnController(const std::string& node_name)
  : Node(node_name) {
  std::string topic_name;

  // int iterator{0};
  for (int i = 0; i < 20; i++) {
    topic_name = "/box_bot" + std::to_string(i) + "/cmd_vel";
    std::cout << topic_name;
    publishers_[i] = this->create_publisher
                  <geometry_msgs::msg::Twist>(topic_name, 10);
  }
  
  timer_publisher_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                             std::bind(&SpawnController::
                                                       timer_callback, this));
  callback_counter_ = 0;
}

void SpawnController::timer_callback() {
  // Logging the message in the terminal.
  RCLCPP_INFO(this->get_logger(), "Controlling the bots.");
  bot_velocity_.linear.x = -0.1;

  for (int i = 0; i < 20; i++) {
    if ((i * 5)  <= callback_counter_) {
      bot_velocity_.angular.z = 0.2;
    } else {
      bot_velocity_.angular.z = 0.0;
    }
    publishers_.at(i)->publish(bot_velocity_);
  }

  callback_counter_++;
}
