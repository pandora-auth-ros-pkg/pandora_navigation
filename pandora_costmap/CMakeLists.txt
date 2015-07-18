cmake_minimum_required(VERSION 2.8.3)
project(pandora_costmap)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  dynamic_reconfigure
  roscpp
  sensor_msgs
  tf
  pcl_ros
  roslint
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  config/filter.cfg
  )

find_package(Eigen REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

catkin_package(
#  INCLUDE_DIRS
#    include
  LIBRARIES
  CATKIN_DEPENDS
    rospy
    roscpp
    dynamic_reconfigure
    sensor_msgs
    tf
    pcl_ros
  DEPENDS
    Eigen
)


add_executable(cloud_cropper src/point_cloud_cropper.cpp)
target_link_libraries(cloud_cropper ${catkin_LIBRARIES})

add_executable(
  range_to_point_cloud_converter
  src/range_to_point_cloud_converter.cpp)
target_link_libraries(
  range_to_point_cloud_converter
  ${catkin_LIBRARIES})

FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_PYTHON
       ${PROJECT_SOURCE_DIR}
       scripts/**/*.py
       test/**/*.py
    )
roslint_python(${${PROJECT_NAME}_LINT_PYTHON})

if (CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/unit/map_utils_test.py)
  catkin_add_nosetests(test/unit/obstacle_test.py)
endif()
