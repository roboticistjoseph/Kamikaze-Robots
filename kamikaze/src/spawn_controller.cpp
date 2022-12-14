#include <spawn_controller.h>

SpawnController::SpawnController(const std::string &node_name)
    : Node(node_name) {
    std::string topic_name;
    // int iterator{0};
    for (int i = 0; i < 20; i++)
    {
        topic_name = "/box_bot" + std::to_string(i) + "/cmd_vel";
        std::cout << topic_name;
        publishers_[i] = this->create_publisher<geometry_msgs::msg::Twist>(topic_name, 10);
        // iterator++;        
    }
    // topic_name = "/box_bot" + std::to_string(0) + "/cmd_vel";
    // publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(topic_name, 10);

    // Starting the publisher.
  timer_publisher_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&SpawnController::timer_callback, this));
}

void SpawnController::timer_callback() {
    // Logging the message in the terminal.
  RCLCPP_INFO(this->get_logger(), "Controlling the bots.");
  bot_velocity_.linear.x = 0.1;
  for (int i = 0; i < 20; i++) {
    publishers_.at(i)->publish(bot_velocity_);
  }
}