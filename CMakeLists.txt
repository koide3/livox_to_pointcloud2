cmake_minimum_required(VERSION 3.5.2)
project(livox_to_pointcloud2)

set(CMAKE_CXX_STANDARD 20)

###########
## Build ##
###########

if($ENV{ROS_VERSION} EQUAL 1)
  ###########################
  ## catkin-related (ROS1) ##
  ###########################
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    livox_ros_driver
  )
  catkin_package()

  add_executable(livox_to_pointcloud2_node
    src/livox_to_pointcloud2_ros1.cpp
  )
  target_include_directories(livox_to_pointcloud2_node PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
  )
  target_link_libraries(livox_to_pointcloud2_node
    ${catkin_LIBRARIES}
  )
  add_dependencies(livox_to_pointcloud2_node
    ${catkin_EXPORTED_TARGETS}
  )
elseif($ENV{ROS_VERSION} EQUAL 2)
  ##########################
  ## ament-related (ROS2) ##
  ##########################
  find_package(ament_cmake_auto REQUIRED)
  ament_auto_find_build_dependencies()

  # Component
  ament_auto_add_library(livox_to_pointcloud2 SHARED
    src/livox_to_pointcloud2_ros2.cpp
  )
  rclcpp_components_register_nodes(livox_to_pointcloud2 "livox_to_pointcloud2::LivoxToPointCloud2")

  # Node
  ament_auto_add_executable(livox_to_pointcloud2_node
    src/livox_to_pointcloud2_ros2_node.cpp
  )
  target_include_directories(livox_to_pointcloud2_node PUBLIC
    include
  )
  target_link_libraries(livox_to_pointcloud2_node
    livox_to_pointcloud2
  )

  ament_auto_package()
endif()

