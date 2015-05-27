cmake_minimum_required(VERSION 2.8.3)
project(pandora_move_back_recovery)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  costmap_2d
  nav_core
  pluginlib
  base_local_planner
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

catkin_package(
  
  CATKIN_DEPENDS
    roscpp
    pluginlib

  INCLUDE_DIRS include
  
  LIBRARIES pandora_move_back_recovery  
)

add_library(pandora_move_back_recovery src/pandora_move_back_recovery.cpp)
target_link_libraries(pandora_move_back_recovery ${catkin_LIBRARIES})