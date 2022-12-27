#pragma once

#if 0
#include <sensor_msgs/PointCloud2.h>
using PointCloud2 = sensor_msgs::PointCloud2;
#else
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <livox_interfaces/msg/custom_msg.hpp>
using PointField = sensor_msgs::msg::PointField;
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointCloud2Ptr = sensor_msgs::msg::PointCloud2::SharedPtr;
using PointCloud2ConstPtr = sensor_msgs::msg::PointCloud2::ConstSharedPtr;
using CustomMsg = livox_interfaces::msg::CustomMsg;
#endif

namespace livox_to_pointcloud2 {

class LivoxConverter {
public:
  LivoxConverter() {
    points_msg.reset(new PointCloud2);

    const auto add_field = [this](const std::string& name, const int offset, const uint8_t datatype) {
      PointField field;
      field.name = name;
      field.offset = offset;
      field.datatype = datatype;
      field.count = 1;
      points_msg->fields.push_back(field);
    };

    add_field("x", 0, PointField::FLOAT32);
    add_field("y", points_msg->fields.back().offset + sizeof(float), PointField::FLOAT32);
    add_field("z", points_msg->fields.back().offset + sizeof(float), PointField::FLOAT32);
    add_field("t", points_msg->fields.back().offset + sizeof(float), PointField::UINT32);
    add_field("intensity", points_msg->fields.back().offset + sizeof(std::uint32_t), PointField::FLOAT32);
    add_field("tag", points_msg->fields.back().offset + sizeof(float), PointField::UINT8);
    add_field("line", points_msg->fields.back().offset + sizeof(std::uint8_t), PointField::UINT8);
    points_msg->is_bigendian = false;
    points_msg->point_step = sizeof(float) * 4 + sizeof(uint32_t) + sizeof(uint8_t) * 2;
    points_msg->is_dense = true;
  }

  PointCloud2ConstPtr convert(const CustomMsg& livox_msg) {
    points_msg->header = livox_msg.header;
    points_msg->width = livox_msg.point_num;
    points_msg->height = 1;

    points_msg->row_step = livox_msg.point_num * points_msg->point_step;
    points_msg->data.resize(points_msg->row_step);

    unsigned char* ptr = points_msg->data.data();
    for (int i = 0; i < livox_msg.point_num; i++) {
      *reinterpret_cast<float*>(ptr + points_msg->fields[0].offset) = livox_msg.points[i].x;
      *reinterpret_cast<float*>(ptr + points_msg->fields[1].offset) = livox_msg.points[i].y;
      *reinterpret_cast<float*>(ptr + points_msg->fields[2].offset) = livox_msg.points[i].z;
      *reinterpret_cast<std::uint32_t*>(ptr + points_msg->fields[3].offset) = livox_msg.points[i].offset_time;
      *reinterpret_cast<float*>(ptr + points_msg->fields[4].offset) = livox_msg.points[i].reflectivity;
      *reinterpret_cast<std::uint8_t*>(ptr + points_msg->fields[5].offset) = livox_msg.points[i].tag;
      *reinterpret_cast<std::uint8_t*>(ptr + points_msg->fields[6].offset) = livox_msg.points[i].line;

      ptr += points_msg->point_step;
    }

    return points_msg;
  }

private:
  PointCloud2Ptr points_msg;
};

}