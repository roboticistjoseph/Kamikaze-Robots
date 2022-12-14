#ifndef KAMIKAZE_INCLUDE_KAMIKAZE_SPAWN_CONTROLLER_H_
#define KAMIKAZE_INCLUDE_KAMIKAZE_SPAWN_CONTROLLER_H_

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <memory>

class SpawnController : public rclcpp::Node {
 public:
  SpawnController(const std::string &node_name = "spawn_controller");

 private:
  std::array<rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr, 20> publishers_;
//   rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;  //!< The pointer to the publisher.
  geometry_msgs::msg::Twist bot_velocity_;  //!< The calculated bot velocity.
  rclcpp::TimerBase::SharedPtr timer_publisher_;  //!< The pointer to the publisher callback.
  int callback_counter_;

  /**
   * @brief The callback funciton 
   */
  void timer_callback();
};  // SpawnController

#endif  // KAMIKAZE_INCLUDE_KAMIKAZE_SPAWN_CONTROLLER_H_