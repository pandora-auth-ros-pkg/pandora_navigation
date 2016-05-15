cmake_minimum_required(VERSION 2.8.3)
project(pandora_monstertruck_2dnav)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  ackermann_msgs
  actionlib
  move_base_msgs
  roslint
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    geometry_msgs
    ackermann_msgs
    actionlib
    move_base_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(ackermann_to_twist
  src/ackermann_to_twist.cpp
)
target_link_libraries(
  ackermann_to_twist
  ${catkin_LIBRARIES}
)

add_executable(ackermann_to_twist_node
  src/ackermann_to_twist_node.cpp
)
target_link_libraries(
  ackermann_to_twist_node
  ackermann_to_twist
  ${catkin_LIBRARIES}
)

add_executable(navigation_goals_loop_node
  src/navigation_goals_loop_node.cpp
)
target_link_libraries(
  navigation_goals_loop_node
  ${catkin_LIBRARIES}
)

# To check lint : catkin_make roslint_pandora_monstertruck_2dnav
set(ROSLINT_CPP_OPTS
  "--filter=-build/include_what_you_use")

file(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}
  *.h
  *.cpp
)
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})
