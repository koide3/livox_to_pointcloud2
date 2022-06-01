cmake_minimum_required(VERSION 3.0.2)
project(livox_to_pointcloud2)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs livox_ros_driver)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES livox_to_pointcloud2
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(livox_to_pointcloud2
  src/livox_to_pointcloud2.cpp
)
target_link_libraries(livox_to_pointcloud2
  ${catkin_LIBRARIES}
)
