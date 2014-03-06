/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, P.A.N.D.O.R.A. Team.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the P.A.N.D.O.R.A. Team nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Manos Tsardoulias
* Author: Aris Thallas
*********************************************************************/

#include "pandora_path_planner/path_generator/tree_path_generator/tree_path_generator.h"
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


