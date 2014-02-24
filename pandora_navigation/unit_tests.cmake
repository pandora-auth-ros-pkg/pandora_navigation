find_package(Boost REQUIRED COMPONENTS serialization)


#~ add_library(serialization_IO tests/serialization_IO.cpp)
#~ target_link_libraries(serialization_IO ${Boost_LIBRARIES})

#~ ####################### topological unit_test ########################
add_executable(topological_serialize
	tests/unit_tests/topological_graph_test/topological_graph_serialize.cpp
)
target_link_libraries(topological_serialize ${catkin_LIBRARIES} 
											${Boost_LIBRARIES}
											voronoi_nodes
											voronoi
											map_attributes
											pixelcoords
											transformation
											coverage
											node
											test_utilities
											Image
												)
												
catkin_add_gtest(topological_deserialize

	tests/unit_tests/topological_graph_test/topological_graph_deserialize.cpp
)
target_link_libraries(topological_deserialize ${catkin_LIBRARIES} 
											${Boost_LIBRARIES}
											voronoi_nodes
											voronoi
											map_attributes
											pixelcoords
											transformation
											coverage
											node
											test_utilities
											Image
												)
												
												
######################## ######################## ########################												
