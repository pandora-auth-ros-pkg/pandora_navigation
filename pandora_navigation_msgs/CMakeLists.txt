cmake_minimum_required(VERSION 2.8.3)
project(pandora_navigation_msgs)
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  actionlib_msgs
)

add_action_files(
  FILES
  DoExploration.action
)

generate_messages(
  DEPENDENCIES
    std_msgs
    actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
    actionlib_msgs
)
