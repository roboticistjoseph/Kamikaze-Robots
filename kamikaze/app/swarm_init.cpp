#include <spawn_controller.h>

#include <memory>

int main(int argc, char* argv[]) {
  // Initializing the rclcpp
  rclcpp::init(argc, argv);

  // Instantiating and spinning the node
  rclcpp::spin(std::make_shared<SpawnController>());

  // Shutdown
  rclcpp::shutdown();

  return 0;
}