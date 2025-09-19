#include <rclcpp/rclcpp.hpp>
#include "kobu_zed_driver/zed_camera_driver.hpp"

int main(int argc, char * argv[])
{
  // Initialize ROS2
  rclcpp::init(argc, argv);
  
  // Set executor options for better real-time performance
  rclcpp::ExecutorOptions options;
  options.memory_strategy = rclcpp::memory_strategies::create_default_strategy();
  
  // Create single-threaded executor for deterministic behavior
  rclcpp::executors::SingleThreadedExecutor executor(options);
  
  // Create the ZED camera driver node
  auto node = std::make_shared<kobu_zed_driver::ZEDCameraDriver>();
  
  // Initialize the node (CRITICAL - must be called!)
  node->initialize();
  
  // Add node to executor
  executor.add_node(node);
  
  RCLCPP_INFO(node->get_logger(), "ZED Camera Node started");
  
  // Spin the executor
  executor.spin();
  
  // Cleanup
  rclcpp::shutdown();
  
  return 0;
}