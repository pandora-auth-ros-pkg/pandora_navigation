# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(pandora_move_base)
find_package(catkin REQUIRED COMPONENTS 
	roscpp
	geometry_msgs
	nav_msgs
	actionlib
	tf
	pandora_path_planner
	pandora_navigation_common
	pandora_navigation_communications
	move_base_msgs
	sensor_msgs
	controllers_and_sensors_communications
)


catkin_package(
    DEPENDS 
    CATKIN_DEPENDS
			roscpp
			geometry_msgs
			nav_msgs
			actionlib
			tf
			pandora_path_planner
			pandora_navigation_common
			pandora_navigation_communications
			move_base_msgs
			sensor_msgs
			controllers_and_sensors_communications
    INCLUDE_DIRS include
    LIBRARIES 
)


include_directories(include
  ${catkin_INCLUDE_DIRS}
)

#################### navigator ######################
add_library(pandora_navigator
	src/navigator/pandora_navigator.cpp
)
target_link_libraries(pandora_navigator ${catkin_LIBRARIES} )

################# navigation ########################
add_library(navigation_controller
	src/navigation_controller.cpp
)
target_link_libraries(navigation_controller ${catkin_LIBRARIES} )


add_executable(move_base_node
	src/move_base_node.cpp
)
target_link_libraries(move_base_node ${catkin_LIBRARIES}
	pandora_navigator
	navigation_controller
)
add_dependencies(move_base_node pandora_navigation_communications_gencpp)
