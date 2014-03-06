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

#include "pandora_path_planner/pandora_path_planner.h"

//~ PandoraPathPlanner::PandoraPathPlanner(void):  partitionGraphPathGenerator( &(_mapUpdater.getMapAttributes() ) ),
											   //~ treePathGenerator( &(_mapUpdater.getMapAttributes() ) ),
											   //~ voronoiPathGenerator( &(_mapUpdater.getMapAttributes() ) )
//~ {
	//~ mapAttributes = & (_mapUpdater.getMapAttributes() );
	//~ 
	//~ _mapUpdater.startSubscriber();
	
//~ }

PandoraPathPlanner::PandoraPathPlanner(MapAttributes* mapAttr): partitionGraphPathGenerator(mapAttr,&voronoi), treePathGenerator(mapAttr,&voronoi), voronoiPathGenerator(mapAttr,&voronoi), voronoi(mapAttr,false) {
	
	mapAttributes = mapAttr;
	
}


bool PandoraPathPlanner::makePlan( const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan ){
	
	voronoi.fixVoronoi();
	
	plan.clear();
	
	//move with partitionGraph
	ROS_INFO_NAMED("path_planner", "[path_planner %d]: Calculating path with partition graph ", __LINE__);
	if ( !partitionGraphPathGenerator.generatePath( goal, plan ) ){
		ROS_INFO_NAMED("path_planner", "[path_planner %d]: Partition path generator CAN NOT find path ", __LINE__);
		
		
		//move with RRT tree
		ROS_INFO_NAMED("path_planner", "[path_planner %d]: Alternative: move with  RRT tree ", __LINE__);
		std::cout<<"Alternative: move with  RRT tree\n";
		if ( !treePathGenerator.generatePath( goal, plan) ){
			ROS_INFO_NAMED("path_planner", "[path_planner %d]: RRT tree path generator CAN NOT find path ", __LINE__);
			
			
			//move with Voronoi
			ROS_INFO_NAMED("path_planner", "[path_planner %d]: Final solution: move with Voronoi ", __LINE__);
			if ( !voronoiPathGenerator.generatePath( goal, plan) ){
				ROS_INFO_NAMED("path_planner", "[path_planner %d]: voronoiPathGenerator CAN NOT find path ", __LINE__);
				ROS_ERROR_NAMED("path_planner", "[path_planner %d]: RUN FOR YOUR LIVES!!! ", __LINE__);
				
				plan.clear();
				return false;
			}
			else
				ROS_INFO_NAMED("path_planner", "[path_planner %d]: Path found with Voronoi! ", __LINE__);
		}
		else
			ROS_INFO_NAMED("path_planner", "[path_planner %d]: Path found with RRT Tree! ", __LINE__);
	}
	else
		ROS_INFO_NAMED("path_planner", "[path_planner %d]: Path found with Partition Graph! ", __LINE__);
	
	//~ pixelPlan.clear();
	//~ for (unsigned int i=0; i, plan.size(); i++)
		//~ pixelPlan.push_back( transformPoseStamped2PixelCoords(plan[i]) );
	
	return true;
}


//to be used ONLY in TargetSelector
void PandoraPathPlanner::getTargetSelectorPaths(NodesVector* nodes){
	voronoi.fixVoronoi();
	ROS_INFO_NAMED("path_planner", "[path_planner %d]: Generating paths for multiple targets ", __LINE__);

	partitionGraphPathGenerator.generatePath( nodes );

}




