#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver/CustomMsg.h>

class LivoxToPointCloud2 {
public:
  LivoxToPointCloud2() : nh("~") {
    points_msg.reset(new sensor_msgs::PointCloud2);

    const auto add_field = [this](const std::string& name, const int offset, const uint8_t datatype) {
      sensor_msgs::PointField field;
      field.name = name;
      field.offset = offset;
      field.datatype = datatype;
      field.count = 1;
      points_msg->fields.push_back(field);
    };

    add_field("x", 0, sensor_msgs::PointField::FLOAT32);
    add_field("y", points_msg->fields.back().offset + sizeof(float), sensor_msgs::PointField::FLOAT32);
    add_field("z", points_msg->fields.back().offset + sizeof(float), sensor_msgs::PointField::FLOAT32);
    add_field("t", points_msg->fields.back().offset + sizeof(float), sensor_msgs::PointField::UINT32);
    add_field("intensity", points_msg->fields.back().offset + sizeof(std::uint32_t), sensor_msgs::PointField::FLOAT32);
    add_field("tag", points_msg->fields.back().offset + sizeof(float), sensor_msgs::PointField::UINT8);
    add_field("line", points_msg->fields.back().offset + sizeof(std::uint8_t), sensor_msgs::PointField::UINT8);
    points_msg->is_bigendian = false;
    points_msg->point_step = sizeof(float) * 4 + sizeof(uint32_t) + sizeof(uint8_t) * 2;
    points_msg->is_dense = true;

    points_pub = nh.advertise<sensor_msgs::PointCloud2>("/livox/points", 10);
    points_sub = nh.subscribe("/livox/lidar", 10, &LivoxToPointCloud2::callback, this);
  }

  void callback(const livox_ros_driver::CustomMsg::ConstPtr& livox_msg) {
    points_msg->header = livox_msg->header;
    points_msg->width = livox_msg->point_num;
    points_msg->height = 1;

    points_msg->row_step = livox_msg->point_num * points_msg->point_step;
    points_msg->data.resize(points_msg->row_step);

    unsigned char* ptr = points_msg->data.data();
    for (int i = 0; i < livox_msg->point_num; i++) {
      *reinterpret_cast<float*>(ptr + points_msg->fields[0].offset) = livox_msg->points[i].x;
      *reinterpret_cast<float*>(ptr + points_msg->fields[1].offset) = livox_msg->points[i].y;
      *reinterpret_cast<float*>(ptr + points_msg->fields[2].offset) = livox_msg->points[i].z;
      *reinterpret_cast<std::uint32_t*>(ptr + points_msg->fields[3].offset) = livox_msg->points[i].offset_time;
      *reinterpret_cast<float*>(ptr + points_msg->fields[4].offset) = livox_msg->points[i].reflectivity;
      *reinterpret_cast<std::uint8_t*>(ptr + points_msg->fields[5].offset) = livox_msg->points[i].tag;
      *reinterpret_cast<std::uint8_t*>(ptr + points_msg->fields[6].offset) = livox_msg->points[i].line;

      ptr += points_msg->point_step;
    }

    points_pub.publish(points_msg);
  }

private:
  ros::NodeHandle nh;
  ros::Subscriber points_sub;
  ros::Publisher points_pub;

  sensor_msgs::PointCloud2::Ptr points_msg;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "livox_to_pointcloud2");
  LivoxToPointCloud2 converter;
  ros::spin();

  return 0;
}