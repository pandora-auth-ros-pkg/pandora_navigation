cmake_minimum_required(VERSION 2.8.3)
project(pandora_costmap)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  sensor_msgs
  tf
  pcl_ros
)

find_package(Eigen REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN_INCLUDE_DIRS}
  LIBRARIES
  CATKIN_DEPENDS
    cmake_modules
    roscpp
    sensor_msgs
    tf
    pcl_ros
  DEPENDS
    Eigen
)


add_executable(cloud_cropper src/point_cloud_cropper.cpp)
target_link_libraries(cloud_cropper ${catkin_LIBRARIES})
