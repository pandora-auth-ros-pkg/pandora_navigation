cmake_minimum_required(VERSION 2.8.3)
project(pandora_target_selector)
find_package(catkin REQUIRED COMPONENTS 
	roscpp
	target_selector_communications
	data_fusion_communications
	pandora_navigation_communications
	geometry_msgs
	std_msgs
	nav_msgs
	std_srvs
	actionlib
	tf
	pandora_path_planner
	pandora_navigation_common
)


catkin_package(
    DEPENDS 
    CATKIN_DEPENDS
			roscpp
			target_selector_communications
			data_fusion_communications
			pandora_navigation_communications
			geometry_msgs
			std_msgs
			nav_msgs
			std_srvs
			actionlib
			tf
			pandora_path_planner
			pandora_navigation_common
    INCLUDE_DIRS include
    LIBRARIES 
)


include_directories(include
  ${catkin_INCLUDE_DIRS}
)


################# target_selector ########################
add_library(target_selector
	src/target_selector.cpp
)
add_dependencies(target_selector
	target_selector_communications_gencpp 
	data_fusion_communications_gencpp
	pandora_navigation_communications_gencpp
)
target_link_libraries(target_selector ${catkin_LIBRARIES} )
 
add_library(exploration_target_selector
	src/exploration_target_selector.cpp
)
target_link_libraries(exploration_target_selector 
		${catkin_LIBRARIES} 
		target_selector)

add_library(closest_unexplored_target_selector
	src/closest_unexplored_target_selector.cpp
)
target_link_libraries(closest_unexplored_target_selector ${catkin_LIBRARIES} )

add_library(victim_target_selector
	src/victim_target_selector.cpp
)
target_link_libraries(victim_target_selector ${catkin_LIBRARIES} )

add_library(target_selector_controller
	src/target_selector_controller.cpp
)
target_link_libraries(target_selector_controller ${catkin_LIBRARIES} )
############################################################




add_executable(target_selector_node
	src/target_selector_node.cpp
)
target_link_libraries(target_selector_node ${catkin_LIBRARIES}
				target_selector
				exploration_target_selector
				victim_target_selector
				target_selector_controller
				closest_unexplored_target_selector
)

