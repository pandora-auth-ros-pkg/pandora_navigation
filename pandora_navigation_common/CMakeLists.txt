cmake_minimum_required(VERSION 2.8.3)
project(pandora_navigation_common)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  cv_bridge
  image_transport
  pandora_navigation_communications
)

catkin_package(
    DEPENDS 
    CATKIN_DEPENDS
      roscpp
      cv_bridge
      image_transport
      pandora_navigation_communications
    INCLUDE_DIRS include
    LIBRARIES 
      voronoi
      node
      voronoi_nodes
      partition_graph_nodes
      map_attributes
      transformation
      coverage
      map_updater
      pixelcoords
)


include_directories(include
  ${catkin_INCLUDE_DIRS}
)

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
target_link_libraries(partition_graph_nodes 
    ${catkin_LIBRARIES} 
    node)
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


####################### misc ########################
add_library(pixelcoords
	src/misc/pixelcoords.cpp
)
target_link_libraries(pixelcoords ${catkin_LIBRARIES} )

###########################################################

add_executable(navigation_streaming src/navigation_map_streaming/streamer.cpp)
target_link_libraries(navigation_streaming ${catkin_LIBRARIES} )
add_dependencies(navigation_streaming pandora_navigation_communications_gencpp)
