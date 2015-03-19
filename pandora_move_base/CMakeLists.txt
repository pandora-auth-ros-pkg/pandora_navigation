cmake_minimum_required(VERSION 2.8.3)
project(pandora_move_base)

find_package(catkin REQUIRED
    COMPONENTS
      cmake_modules
      roscpp
      pluginlib
      actionlib
      dynamic_reconfigure
      nav_core
      tf
      visualization_msgs
      move_base_msgs
      nav_core
      geometry_msgs
)


find_package(Eigen REQUIRED)
add_definitions(${EIGEN_DEFINITIONS})

# dynamic reconfigure
generate_dynamic_reconfigure_options(
    config/MoveBase.cfg
)

catkin_package(
#  DEPENDS
  CATKIN_DEPENDS
    roscpp
    pluginlib
    actionlib
    dynamic_reconfigure
    nav_core
    tf
    visualization_msgs
    move_base_msgs
    nav_core
    geometry_msgs
#  INCLUDE_DIRS
#    include
#  LIBRARIES
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
)

add_library(pandora_move_base src/move_base.cpp)
target_link_libraries(pandora_move_base ${catkin_LIBRARIES})
add_dependencies(pandora_move_base ${PROJECT_NAME}_gencfg)

add_executable(pandora_move_base_node
  src/move_base_node.cpp
)
target_link_libraries(pandora_move_base_node pandora_move_base)


