#include <iostream>
#include <Eigen/Core>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

class KinectToLiDAR {
public:
  KinectToLiDAR() : nh("~") {
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
    add_field("intensity", points_msg->fields.back().offset + sizeof(float), sensor_msgs::PointField::FLOAT32);
    add_field("ring", points_msg->fields.back().offset + sizeof(float), sensor_msgs::PointField::UINT16);
    add_field("time", points_msg->fields.back().offset + sizeof(std::uint16_t), sensor_msgs::PointField::FLOAT32);
    points_msg->is_bigendian = false;
    points_msg->point_step = sizeof(float) * 5 + sizeof(std::uint16_t);
    points_msg->is_dense = true;

    points_pub = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 10);
    points_sub = nh.subscribe("/points2", 10, &KinectToLiDAR::callback, this);
  }

  void callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    points_msg->header = msg->header;

    std::vector<Eigen::Vector3f> points;
    for (int i = 0; i < msg->width * msg->height; i++) {
      const std::uint8_t* data = msg->data.data() + i * msg->point_step;

      const Eigen::Vector3f pt = Eigen::Map<const Eigen::Vector3f>(reinterpret_cast<const float*>(data));
      if(!pt.array().isFinite().all()) {
        continue;
      }

      points.emplace_back(pt);
    }

    points_msg->width = points.size();
    points_msg->height = 1;

    points_msg->row_step = points.size() * points_msg->point_step;
    points_msg->data.resize(points_msg->row_step);

    unsigned char* ptr = points_msg->data.data();
    for (int i = 0; i < points.size(); i++) {
      *reinterpret_cast<float*>(ptr + points_msg->fields[0].offset) = points[i].x();
      *reinterpret_cast<float*>(ptr + points_msg->fields[1].offset) = points[i].y();
      *reinterpret_cast<float*>(ptr + points_msg->fields[2].offset) = points[i].z();
      *reinterpret_cast<float*>(ptr + points_msg->fields[3].offset) = 0.0f;
      *reinterpret_cast<std::uint16_t*>(ptr + points_msg->fields[4].offset) = 0;
      *reinterpret_cast<float*>(ptr + points_msg->fields[5].offset) = 0.0f;

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
  ros::init(argc, argv, "kinect_to_lidar");
  KinectToLiDAR converter;
  ros::spin();

  return 0;
}