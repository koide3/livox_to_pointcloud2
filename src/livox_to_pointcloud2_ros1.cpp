#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver/CustomMsg.h>

#define ROS1
#include <livox_to_pointcloud2/livox_converter.hpp>

namespace livox_to_pointcloud2 {

class LivoxToPointCloud2 {
public:
  LivoxToPointCloud2() : nh("~") {
    points_pub = nh.advertise<sensor_msgs::PointCloud2>("/livox/points", 10);
    points_sub = nh.subscribe("/livox/lidar", 10, &LivoxToPointCloud2::callback, this);
  }

  void callback(const livox_ros_driver::CustomMsg::ConstPtr& livox_msg) {
    const auto points_msg = converter.convert(*livox_msg);
    points_pub.publish(points_msg);
  }

private:
  ros::NodeHandle nh;
  ros::Subscriber points_sub;
  ros::Publisher points_pub;

  LivoxConverter converter;
};

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "livox_to_pointcloud2");
  livox_to_pointcloud2::LivoxToPointCloud2 node;
  ros::spin();

  return 0;
}