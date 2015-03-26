add_executable(frontier_test frontier_selector_test.cpp)
target_link_libraries(frontier_test
  ${catkin_LIBRARIES}
  frontier_goal_selector
)


catkin_add_gtest(costmap_tools_test unit/costmap_tools_test.cpp)
target_link_libraries(costmap_tools_test
  ${catkin_LIBRARIES}
  gtest_main
  )


catkin_add_gtest(map_frontier_search_test unit/map_frontier_search_test.cpp)
target_link_libraries(map_frontier_search_test
  ${catkin_LIBRARIES}
  gtest_main
  frontier_searchers
  )

catkin_add_gtest(export_costmap export_costmap.cpp)
target_link_libraries(export_costmap
  ${catkin_LIBRARIES}
  gtest_main
  )
