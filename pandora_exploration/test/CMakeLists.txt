add_executable(frontier_test frontier_selector_test.cpp)
target_link_libraries(frontier_test
  ${catkin_LIBRARIES}
  frontier_goal_selector
)
