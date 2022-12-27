# livox_to_pointcloud2

This package provides ROS1/ROS2 interfaces to convert ```livox_interfaces::msg::CustomMsg``` into ```sensor_msgs::msg::PointCloud2```.

## Usage

```bash
# Launch as a ROS1 node
rosrun livox_to_pointcloud2 livox_to_pointcloud2_node

# Launch as a ROS2 node
ros2 run livox_to_pointcloud2 livox_to_pointcloud2_node

# Launch as a standalone ROS2 component
ros2 component standalone livox_to_pointcloud2 livox_to_pointcloud2::LivoxToPointCloud2
```

- In/Out Topics:
  - Input topic: **/livox/lidar** (```livox_interfaces::msg::CustomMsg```)
  - Output topic: **/livox/points** (```sensor_msgs::msg::PointCloud2```)

- Point fields in the output pointcloud:
  - x, y, z : FLOAT32
  - t : UINT32
  - intensity : FLOAT32
  - tag : UINT8
  - line : UINT8

## License

This package is released under the MIT license.
