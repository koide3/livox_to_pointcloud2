#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <livox_to_pointcloud2/livox_to_pointcloud2_ros2.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<livox_to_pointcloud2::LivoxToPointCloud2>(options));
  return 0;
}