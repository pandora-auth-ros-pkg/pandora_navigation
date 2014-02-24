####################### move_base_client ########################
add_executable(move_base_client_cpp
	tests/move_base_client_cpp.cpp 
)
target_link_libraries(move_base_client_cpp ${catkin_LIBRARIES} )
 ######################## ##################
 
####################### dummy_fsm ########################
add_executable(dummy_fsm
	tests/dummy_fsm.cpp 
)
target_link_libraries(dummy_fsm ${catkin_LIBRARIES} )
 ######################## ##################


####################### image object ########################
add_library(Image
			tests/Image.cpp 
)
target_link_libraries(Image ${catkin_LIBRARIES})
####################### image object ########################

####################### image object ########################
add_library(test_utilities
			tests/test_utilities.cpp 
)
target_link_libraries(test_utilities ${catkin_LIBRARIES})
####################### image object ########################

####################### voronoi manual test ########################
add_executable(voronoi_test
	tests/voronoi_test/voronoi_test.cpp 
)
target_link_libraries(voronoi_test ${catkin_LIBRARIES} 
									voronoi 
									map_attributes
									pixelcoords
									transformation
									test_utilities
									Image
									  )
 ######################## ######################## ########################

####################### topological graph manual test ########################
add_executable(topological_graph_test
	tests/topological_graph_test/topological_graph_test.cpp
)
target_link_libraries(topological_graph_test ${catkin_LIBRARIES}
											voronoi_nodes
											voronoi
											map_attributes
											node
											pixelcoords
											transformation
											coverage
											test_utilities
											Image
												)
												
######################## ######################## ######################## 										
										

####################### partition graph manual test ########################												
add_executable(partition_graph_test

	tests/partition_graph_test/partition_graph_test.cpp

)
target_link_libraries(partition_graph_test ${catkin_LIBRARIES}
											map_attributes
											pixelcoords
											transformation
											partition_graph_nodes
											voronoi
											node
											test_utilities
											Image
												)
												
####################### ####################### ####################### 												
												
												
####################### rrt planner manual test ########################												
add_executable(rrt_planner_test

			tests/path_planner_test/rrt_planner_test.cpp 


)
target_link_libraries(rrt_planner_test ${catkin_LIBRARIES}
											tree_path_generator
											map_attributes
											pixelcoords
											transformation
											voronoi
											rrt_tree
											tree_node
											path_generator
											test_utilities
											Image
												)

################################################ ########################											


####################### voronoi planner manual test ########################
add_executable(voronoi_planner_test

			tests/path_planner_test/voronoi_planner_test.cpp 
			src/navigation/path_planner/path_generator/voronoi_path_generator/voronoi_path_generator.cpp 

)
target_link_libraries(voronoi_planner_test ${catkin_LIBRARIES}
											map_attributes
											pixelcoords
											transformation
											voronoi
											node
											path_generator
											test_utilities
											Image
												)

################################################ ########################

####################### partition graph planner manual test ########################
add_executable(partition_graph_planner_test

			tests/path_planner_test/partition_graph_planner_test.cpp 
)
target_link_libraries(partition_graph_planner_test ${catkin_LIBRARIES}
											partition_graph_path_generator
											map_attributes
											pixelcoords
											transformation
											voronoi
											dijkstra
											path_generator
											partition_graph_nodes
											node
											test_utilities
											Image
												)

################################################ ########################

####################### pandora path planner manual test ########################
add_executable(pandora_path_planner_test

			tests/path_planner_test/pandora_path_planner_test.cpp 
)
target_link_libraries(pandora_path_planner_test ${catkin_LIBRARIES}
											pandora_path_planner
											tree_path_generator
											voronoi_path_generator
											partition_graph_path_generator
											map_attributes
											pixelcoords
											transformation
											voronoi
											dijkstra
											path_generator
											partition_graph_nodes
											rrt_tree
											tree_node
											node
											test_utilities
											Image
												)

################################################ ########################


####################### closest unexplored target selector manual test ########################
add_executable(closest_unexplored_target_selector_test

			tests/Image.cpp 
			tests/closest_unexplored_target_selector_test/closest_unexplored_target_selector_test.cpp 
)
target_link_libraries(closest_unexplored_target_selector_test ${catkin_LIBRARIES}
											closest_unexplored_target_selector
											target_selector
											map_attributes
											pixelcoords
											transformation
											target_selector
											coverage
											test_utilities
											Image
												)

################################################ ########################


####################### exploration target selector manual test ########################
add_executable(exploration_target_selector_test

			tests/Image.cpp 
			tests/exploration_target_selector_test/main.cpp 
)
target_link_libraries(exploration_target_selector_test ${catkin_LIBRARIES}
											exploration_target_selector
											closest_unexplored_target_selector
											target_selector
											map_attributes
											pandora_path_planner
											tree_path_generator
											voronoi_path_generator
											partition_graph_path_generator
											partition_graph_path_generator
											partition_graph_nodes
											dijkstra
											voronoi_nodes
											pixelcoords
											transformation
											path_generator
											target_selector
											voronoi
											coverage
											rrt_tree
											tree_node
											node
											Image
												)

################################################ ########################
