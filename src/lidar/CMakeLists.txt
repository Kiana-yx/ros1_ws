cmake_minimum_required(VERSION 3.0.2)
project(lidar)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV REQUIRED
  pcl_ros
)

find_package(PCL 1.7 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros
)


include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# 添加头文件路径
link_directories(
  ${PCL_LIBRARY_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/lidar_node.cpp src/lidar_core.cpp)
add_executable(united src/united_node.cpp)

# 链接库
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_INCLUDE_DIRS}
)

# 链接库
target_link_libraries(united
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_INCLUDE_DIRS}
)

