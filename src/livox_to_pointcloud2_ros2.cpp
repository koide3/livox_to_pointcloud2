#include <livox_to_pointcloud2/livox_to_pointcloud2_ros2.hpp>

#include <rclcpp_components/register_node_macro.hpp>

namespace livox_to_pointcloud2 {

LivoxToPointCloud2::LivoxToPointCloud2(const rclcpp::NodeOptions& options) : rclcpp::Node("livox_to_pointcloud2", options) {
  points_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/livox/points", rclcpp::SensorDataQoS());
  livox_sub = this->create_subscription<CustomMsg>("/livox/lidar", rclcpp::SensorDataQoS(), [this](const CustomMsg::ConstSharedPtr livox_msg) {
    const auto points_msg = converter.convert(*livox_msg);
    points_pub->publish(*points_msg);
  });
}

LivoxToPointCloud2::~LivoxToPointCloud2() {}

}  // namespace livox_to_pointcloud2

RCLCPP_COMPONENTS_REGISTER_NODE(livox_to_pointcloud2::LivoxToPointCloud2);