#include "navigation/path_planner/path_generator/tree_path_generator/tree_path_generator.h"
#include "ros/ros.h"

TreePathGenerator::TreePathGenerator(MapAttributes* mapAttr,Voronoi *v):
					PathGenerator(mapAttr,v),
					robotTree( PixelCoords((*mapAttr).robotPose.dx, (*mapAttr).robotPose.dy), mapAttr, v) {
	
	mapAttributes = mapAttr;
	
	pixelPlan.clear();
	
}


bool TreePathGenerator::generatePath(const geometry_msgs::PoseStamped& poseStampedGoal, std::vector<geometry_msgs::PoseStamped>& poseStampedPlan){
	
	pixelPlan.clear();
	
	robotPosition = (PixelCoords (mapAttributes->robotPose.dx +START_X, mapAttributes->robotPose.dy+START_Y) );
	pixelGoal = transformPoseStamped2PixelCoords( poseStampedGoal );
	ROS_INFO_NAMED("tree_path_generator","[tree_path_generator %d]: Creating RRT tree",__LINE__);
	createSimpleRRT( robotPosition, pixelGoal, pixelPlan );
	
	//TODO: check if pushing final target position is needed
	poseStampedPlan.clear();
	if ( pixelPlan.size()!=0 ){
		ROS_INFO_NAMED("tree_path_generator","[tree_path_generator %d]: Minimizing path",__LINE__);
		minimizePath( robotPosition, pixelGoal, pixelPlan );
		
		for (unsigned int i=0; i< pixelPlan.size(); i++)
			poseStampedPlan.push_back( transformPixelCoords2PoseStamped(pixelPlan[i]) );
		poseStampedPlan.push_back( transformPixelCoords2PoseStamped(pixelGoal) );
		return true;
	}
	else{
		ROS_INFO_NAMED("tree_path_generator","[tree_path_generator %d]: Path NOT found",__LINE__);
		return false;
	}
}


/**
	@brief Creates simple RRT
	@param startPos [PixelCoords] : The starting pixel
	@param target [PixelCoords] : The ending pixel
	@return void
	**/				
void TreePathGenerator::createSimpleRRT(PixelCoords startPos, PixelCoords target, std::vector<PixelCoords>& wantedPath ){

	ROS_INFO_NAMED("tree_path_generator","[tree_path_generator %d]: Computing brushcell",__LINE__);

	std::vector<PixelCoords> posPath;
	bool goalFound=false;
	
	posPath.clear();
	robotTree.deleteNodes();
	robotTree=RRTTree(startPos, mapAttributes, voronoi);
	
	long simpleRRTcounter=0;
	ROS_INFO_NAMED("tree_path_generator","[tree_path_generator %d]: Expanding tree",__LINE__);
	while(simpleRRTcounter<3000){
		simpleRRTcounter++;
		//goalFound=robotTree.expand(prevxMax,prevxMin,prevyMax,prevyMin,brushCell,target);
		goalFound=robotTree.expand(target);
		if(goalFound){
			ROS_INFO_NAMED("tree_path_generator","[tree_path_generator %d]: Target found",__LINE__);
			break;
		}
	}
	if(simpleRRTcounter>=3000){
		ROS_INFO_NAMED("tree_path_generator","[tree_path_generator %d]: Target NOT found",__LINE__);
		wantedPath.clear();
		return;
	}
	ROS_INFO_NAMED("tree_path_generator","[tree_path_generator %d]: Choosing path",__LINE__);
	robotTree.choosePath(robotTree.checkNodes.size()-1,posPath);
	
	for(int i=posPath.size()-1;i>=0;i--){
		wantedPath.push_back(posPath[i]);
	}
	
}


