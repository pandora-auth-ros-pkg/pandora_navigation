# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(pandora_navigation)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS roscpp
					actionlib 
					pandora_navigation_communications 
					state_manager 
					tf 
					diagnostic_updater 
					image_transport 
					cv_bridge 
					sensor_msgs
					std_srvs
          move_base_msgs
					)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(CMAKE_BUILD_TYPE Debug)

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    DEPENDS 
    CATKIN-DEPENDS cv_bridge
    INCLUDE_DIRS include
    LIBRARIES test_utilities map_attributes pixelcoords transformation Image 
)


include_directories(include
  ${catkin_INCLUDE_DIRS}
)



#################### navigator ######################
add_library(pandora_navigator
	src/navigation/navigator/pandora_navigator.cpp
)
target_link_libraries(pandora_navigator ${catkin_LIBRARIES} )
####################################################



#################### voronoi ######################
add_library(voronoi
	src/voronoi/voronoi.cpp
)
target_link_libraries(voronoi ${catkin_LIBRARIES} )
####################################################



######################## nodes #########################
add_library(node
	src/nodes/node.cpp
)
target_link_libraries(node ${catkin_LIBRARIES} )

add_library(voronoi_nodes
	src/nodes/voronoi_nodes.cpp
)
target_link_libraries(voronoi_nodes ${catkin_LIBRARIES} )

add_library(partition_graph_nodes
	src/nodes/partition_graph_nodes.cpp
)
target_link_libraries(partition_graph_nodes ${catkin_LIBRARIES} )
####################################################


####################### map ########################
add_library(map_attributes
	src/map/map_attributes.cpp
)
target_link_libraries(map_attributes ${catkin_LIBRARIES} )

add_library(transformation
	src/map/transformation.cpp
)
target_link_libraries(transformation ${catkin_LIBRARIES} )

add_library(coverage
	src/map/coverage.cpp
)
target_link_libraries(coverage ${catkin_LIBRARIES} )

add_library(map_updater
	src/map/map_updater.cpp
)
target_link_libraries(map_updater ${catkin_LIBRARIES} )


###########################################################


################# target_selector ########################
add_library(target_selector
	src/target_selector/target_selector.cpp
)
target_link_libraries(target_selector ${catkin_LIBRARIES} )
 
add_library(exploration_target_selector
	src/target_selector/exploration_target_selector.cpp
)
target_link_libraries(exploration_target_selector ${catkin_LIBRARIES} )

add_library(closest_unexplored_target_selector
	src/target_selector/closest_unexplored_target_selector.cpp
)
target_link_libraries(closest_unexplored_target_selector ${catkin_LIBRARIES} )

add_library(victim_target_selector
	src/target_selector/victim_target_selector.cpp
)
target_link_libraries(victim_target_selector ${catkin_LIBRARIES} )

add_library(target_selector_controller
	src/target_selector/target_selector_controller.cpp
)
target_link_libraries(target_selector_controller ${catkin_LIBRARIES} )
############################################################




################# navigation ########################
add_library(navigation_controller
	src/navigation/navigation_controller.cpp
)
target_link_libraries(navigation_controller ${catkin_LIBRARIES} )


		################# path planner #################
add_library(global_path_planner
	src/navigation/path_planner/global_path_planner.cpp
)
target_link_libraries(global_path_planner ${catkin_LIBRARIES} )

add_library(pandora_path_planner
	src/navigation/path_planner/pandora_path_planner.cpp
)
target_link_libraries(pandora_path_planner ${catkin_LIBRARIES} )
		################# ################# #################


		################# path generator #################
add_library(dijkstra
	src/navigation/path_planner/path_generator/dijkstra.cpp
)
target_link_libraries(dijkstra ${catkin_LIBRARIES} )

add_library(path_generator
	src/navigation/path_planner/path_generator/path_generator.cpp
)
target_link_libraries(path_generator ${catkin_LIBRARIES} )
		################# ################# #################


		################# voronoi gen #################
add_library(voronoi_path_generator
	src/navigation/path_planner/path_generator/voronoi_path_generator/voronoi_path_generator.cpp
)
target_link_libraries(voronoi_path_generator ${catkin_LIBRARIES} )
		################# ################# #################

		################# partition graph gen #################
add_library(partition_graph_path_generator
	src/navigation/path_planner/path_generator/partition_graph_path_generator/partition_graph_path_generator.cpp
)
target_link_libraries(partition_graph_path_generator ${catkin_LIBRARIES} )
		################# ################# #################

		################# tree gen ################# 
add_library(tree_node
	src/navigation/path_planner/path_generator/tree_path_generator/tree_node.cpp
)
target_link_libraries(tree_node ${catkin_LIBRARIES} )

add_library(rrt_tree
	src/navigation/path_planner/path_generator/tree_path_generator/rrt_tree.cpp
)
target_link_libraries(rrt_tree ${catkin_LIBRARIES} )

add_library(tree_path_generator
	src/navigation/path_planner/path_generator/tree_path_generator/tree_path_generator.cpp
)
target_link_libraries(tree_path_generator ${catkin_LIBRARIES} )
		################# ################# #################
############################################################


####################### misc ########################
add_library(pixelcoords
	src/misc/pixelcoords.cpp
)
target_link_libraries(pixelcoords ${catkin_LIBRARIES} )

###########################################################


add_executable(move_base_node
	src/navigation/move_base_node.cpp
)
target_link_libraries(move_base_node ${catkin_LIBRARIES}
						navigation_controller
						pandora_navigator
						map_updater
						pandora_path_planner
						global_path_planner
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
						
)
add_dependencies(target_selector pandora_navigation_communications_gencpp)

add_executable(target_selector_node
	src/target_selector/target_selector_node.cpp
)
target_link_libraries(target_selector_node ${catkin_LIBRARIES}
						exploration_target_selector
						victim_target_selector
						map_updater
						map_attributes
						target_selector_controller
						closest_unexplored_target_selector
						voronoi_nodes
						pandora_path_planner
						global_path_planner
						tree_path_generator
						voronoi_path_generator
						partition_graph_path_generator
						dijkstra
						path_generator
						partition_graph_nodes
						rrt_tree
						tree_node
						target_selector
						transformation
						pixelcoords
						coverage
						voronoi
						node
)
add_dependencies(target_selector pandora_navigation_communications_gencpp)

add_executable(navigation_streaming src/navigation_map_streaming/streamer.cpp)
target_link_libraries(navigation_streaming ${catkin_LIBRARIES} )
add_dependencies(navigation_streaming pandora_navigation_communications_gencpp)

### Tests ###
include(${CMAKE_CURRENT_SOURCE_DIR}/tests.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/unit_tests.cmake)

									
