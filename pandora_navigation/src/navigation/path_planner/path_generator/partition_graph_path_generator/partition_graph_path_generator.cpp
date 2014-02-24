#include "navigation/path_planner/path_generator/partition_graph_path_generator/partition_graph_path_generator.h"
#include "ros/ros.h"
#define FLOW

//~ PartitionGraphPathGenerator::PartitionGraphPathGenerator(void){
PartitionGraphPathGenerator::PartitionGraphPathGenerator(MapAttributes* mapAttr,Voronoi *v): PathGenerator(mapAttr,v), partitionNodes(mapAttr, v) {
	
}

bool PartitionGraphPathGenerator::generatePath(const geometry_msgs::PoseStamped& poseStampedGoal, std::vector<geometry_msgs::PoseStamped>& poseStampedPlan){
	
	pixelGoal = transformPoseStamped2PixelCoords( poseStampedGoal );
	robotPosition = (PixelCoords (mapAttributes->robotPose.dx +START_X, mapAttributes->robotPose.dy+START_Y) );
	
	//~ ROS_ERROR("[Path creator] Goal : %d %d",pixelGoal.getXCoord(),pixelGoal.getYCoord());
	
	ROS_INFO_NAMED("partition_graph_path_generator_map_values","[partition_graph_path_generator %d]: Creating Partition  mapAttributes->robotPose.dx=%d mapAttributes->robotPose.dy=%d",__LINE__ , mapAttributes->robotPose.dx, mapAttributes->robotPose.dy);
	
	//calculate partition graph
	ROS_INFO_NAMED("partition_graph_path_generator","[partition_graph_path_generator %d]: Updating partition graph",__LINE__);
	partitionNodes.createIncrementalPartition( robotPosition, pixelGoal );
	
	//calculate shortest path
	pixelPlan.clear();
	ROS_INFO_NAMED("partition_graph_path_generator","[partition_graph_path_generator %d]: Finding shortest path to target",__LINE__);
	shortestPathFinder.findShortestPathAStar(partitionNodes.nodes[0], partitionNodes.nodes[1], partitionNodes, pixelPlan);
	
	
	poseStampedPlan.clear();
	if ( pixelPlan.size()!=0 ){
		//add target to path
		pixelPlan.push_back(partitionNodes.nodes[1].p);
		
		for (unsigned int i=0; i<pixelPlan.size(); i++)
			poseStampedPlan.push_back( transformPixelCoords2PoseStamped(pixelPlan[i]) );
		poseStampedPlan.push_back( transformPixelCoords2PoseStamped(pixelGoal) );
		return true;
	}
	else return false;
}


bool PartitionGraphPathGenerator::generatePath(NodesVector* nodesVector){ //these are voroNodes
	
	ROS_INFO_NAMED("partition_graph_path_generator","[partition_graph_path_generator %d]: Calculating paths for multiple targets",__LINE__);
	
	//robot position and attributes will be read from mapAttribute object
	robotPosition = PixelCoords(mapAttributes->robotPose.dx+START_X, mapAttributes->robotPose.dy+START_Y);
	ROS_INFO_NAMED("partition_graph_path_generator","[partition_graph_path_generator %d]: Updating partition graph",__LINE__);
	//calculate partition graph
	partitionNodes.createIncrementalPartition( robotPosition, robotPosition );
	
	
	for( unsigned int i=0; i<nodesVector->nodes.size(); i++ ){
		ROS_INFO_NAMED("partition_graph_path_generator","[partition_graph_path_generator %d]: Inserting target in partition graph",__LINE__);
		int tempID = partitionNodes.insertNodeInPartition(nodesVector->nodes[i]);
		
		pixelPlan.clear();
		ROS_INFO_NAMED("partition_graph_path_generator","[partition_graph_path_generator %d]: Finding shortest path to target",__LINE__);
		shortestPathFinder.findShortestPath(partitionNodes.nodes[0], partitionNodes.nodes[tempID], partitionNodes, pixelPlan);
		
		nodesVector->nodes[i].path.clear();
		if (pixelPlan.size()!=0){
			pixelPlan.push_back(nodesVector->nodes[i].p);
			
			ROS_INFO_NAMED("partition_graph_path_generator","[partition_graph_path_generator %d]: Found Path",__LINE__);
			//~ nodesVector->nodes[i].path.clear();
			
			for (unsigned int j=0; j<pixelPlan.size(); j++){
				nodesVector->nodes[i].path.push_back(pixelPlan[j]);
			}
		}
		else
			ROS_INFO_NAMED("partition_graph_path_generator","[partition_graph_path_generator %d]: Path NOT Found ",__LINE__);
			
		ROS_INFO_NAMED("partition_graph_path_generator","[partition_graph_path_generator %d]: Removing target from partition graph",__LINE__);
		partitionNodes.eliminateNode(tempID, true);
	}
	
	return true;
}



