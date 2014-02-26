cmake_minimum_required(VERSION 2.8.3)
project(pandora_path_planner)

find_package(catkin REQUIRED COMPONENTS roscpp
	geometry_msgs
	pandora_navigation_common
	)


catkin_package(
    DEPENDS 
    CATKIN_DEPENDS
			geometry_msgs
			pandora_navigation_common
    INCLUDE_DIRS include
    LIBRARIES
			pandora_path_planner
			global_path_planner
			dijkstra
			path_generator
			voronoi_path_generator
			partition_graph_path_generator
			tree_node
			rrt_tree
			tree_path_generator
)


include_directories(include
  ${catkin_INCLUDE_DIRS}
)

################# path planner #################
add_library(global_path_planner
	src/global_path_planner.cpp
)
target_link_libraries(global_path_planner ${catkin_LIBRARIES} )

add_library(pandora_path_planner
	src/pandora_path_planner.cpp
)
target_link_libraries(pandora_path_planner ${catkin_LIBRARIES} )
################# ################# #################


################# path generator #################
add_library(dijkstra
	src/path_generator/dijkstra.cpp
)
target_link_libraries(dijkstra ${catkin_LIBRARIES} )

add_library(path_generator
	src/path_generator/path_generator.cpp
)
target_link_libraries(path_generator ${catkin_LIBRARIES} )
################# ################# #################


################# voronoi gen #################
add_library(voronoi_path_generator
	src/path_generator/voronoi_path_generator/voronoi_path_generator.cpp
)
target_link_libraries(voronoi_path_generator ${catkin_LIBRARIES} )
################# ################# #################

################# partition graph gen #################
add_library(partition_graph_path_generator
	src/path_generator/partition_graph_path_generator/partition_graph_path_generator.cpp
)
target_link_libraries(partition_graph_path_generator ${catkin_LIBRARIES} )
################# ################# #################

################# tree gen ################# 
add_library(tree_node
	src/path_generator/tree_path_generator/tree_node.cpp
)
target_link_libraries(tree_node ${catkin_LIBRARIES} )

add_library(rrt_tree
	src/path_generator/tree_path_generator/rrt_tree.cpp
)
target_link_libraries(rrt_tree 
		${catkin_LIBRARIES} 
		tree_node)

add_library(tree_path_generator
	src/path_generator/tree_path_generator/tree_path_generator.cpp
)
target_link_libraries(tree_path_generator ${catkin_LIBRARIES} )
################# ################# #################
