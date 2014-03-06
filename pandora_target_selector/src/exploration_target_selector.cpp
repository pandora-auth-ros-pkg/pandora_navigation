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

#include "pandora_target_selector/exploration_target_selector.h"
//~ #define FLOW

ExplorationTargetSelector::ExplorationTargetSelector(MapAttributes& mapAttr, Coverage& cov):
								TargetSelector(mapAttr, cov),
								voronoiNodes(&mapAttr, &cov),
								closestUnexploredTargetSelector(mapAttr, cov),
								pandoraPathPlanner(&mapAttr)
{
	
	prevTarget.setXCoord(0);
	prevTarget.setYCoord(0);
	
}

//~ PixelCoords ExplorationTargetSelector::selectTarget(PixelCoords* target){
void ExplorationTargetSelector::selectTarget(PixelCoords* target){
	
	
	resetTargets();
	
	int currId;
	unsigned int currX;
	unsigned int currY;
	PixelCoords currPos;
	PixelCoords rpose;

	currX=mapAttributes.robotPose.dx+START_X;
	currY=mapAttributes.robotPose.dy+START_Y;
	currPos=PixelCoords(currX, currY);
	rpose=PixelCoords(mapAttributes.robotPose.dx+START_X,mapAttributes.robotPose.dy+START_Y);
	//~ #ifdef FLOW
		//~ ROS_INFO("[Target Selector %d] mapAttributes->robotPose.dx=%d mapAttributes->robotPose.dy=%d",__LINE__ , mapAttributes.robotPose.dx, mapAttributes.robotPose.dy);
		//~ ROS_INFO("[Target Selector %d] mapAttributes->prevxMax=%d mapAttributes->prevxMin=%d mapAttributes->prevyMax=%d mapAttributes->prevyMin=%d",__LINE__ , mapAttributes.prevxMax,mapAttributes.prevxMin,mapAttributes.prevyMax,mapAttributes.prevyMin);
	//~ #endif

	ROS_INFO_NAMED("exploration_target_selector","[exploration_target_selector %d] Computing voronoi nodes",__LINE__);
	voronoiNodes.createVoronodes();
	
	
	for(unsigned int i=0;i<voronoiNodes.nodes.size();i++)
		nodesQueue.insert(voronoiNodes.nodes[i].ID);
	
	if(nodesQueue.size()!=0){
		
		
		bool calculateWeights=true;
		float maxWeight=-1,tempWeight;
		int best=-1;
		bool done=false;
		//~ int currId;
		while(!done){
			currX=mapAttributes.robotPose.dx+START_X;
			currY=mapAttributes.robotPose.dy+START_Y;
			currPos=PixelCoords(currX, currY);
			
			if(nodesQueue.size()==0) 
				break;
			
			done=true;
			
			ROS_INFO_NAMED("exploration_target_selector","[exploration_target_selector %d] Selecting node",__LINE__);
			
			if (calculateWeights){
				calculateWeights=false;
				currId=selectNode(nodesQueue);
			}
			else{
				ROS_INFO_NAMED("exploration_target_selector","[exploration_target_selector %d] Using Pre-Calculated Weights",__LINE__);
				maxWeight=-1;
				best=-1;
				for (std::set<unsigned int>::iterator p=nodesQueue.begin();p!=nodesQueue.end();p++){
					tempWeight=calcNodesWeight(*p);
					if(tempWeight>maxWeight){
						maxWeight=tempWeight;
						best=*p;
					}
				}
				currId = best;
			}
			
			//TODO: check if it returns -1 (probably not)
			
			nodesQueue.erase(currId);
			
			
			if(coverage.coverage[voronoiNodes.nodes[currId].p.getXCoord()][voronoiNodes.nodes[currId].p.getYCoord()]>COVERAGE_LIMITS){
				ROS_INFO_NAMED("exploration_target_selector","[exploration_target_selector %d] Node is eliminated by coverage",__LINE__);
				done=false; //break;
			}
			else{
				goal = voronoiNodes.nodes[currId].path;
			}
		
		}
	}
	
	PixelCoords closestCoverageUnexplored;
	
	ROS_INFO_NAMED("exploration_target_selector","[exploration_target_selector %d] Requesting coverage target from closest_unexplored_target_selector",__LINE__);
	closestUnexploredTargetSelector.selectTarget(&closestCoverageUnexplored);
	closest = closestCoverageUnexplored;
	
	
	//~ if(closestCoverageUnexplored->getXCoord()==-1 || map[closestCoverageUnexplored->getXCoord()][closestCoverageUnexplored->getYCoord()]<=127 ){
	if(closestCoverageUnexplored.getXCoord()==-1 || mapAttributes.map[closestCoverageUnexplored.getXCoord()][closestCoverageUnexplored.getYCoord()]<=127 ){
		
		ROS_ERROR_NAMED("exploration_target_selector","[exploration_target_selector %d] Invalid closestCoverageUnexplored-Cant go with coverage",__LINE__);
	}
	else if(closestCoverageUnexplored.computeDistanceFrom(currPos)>30){
		
		geometry_msgs::PoseStamped start;
		//~ geometry_msgs::PoseStamped target = transformPixelCoords2PoseStamped( closestCoverageUnexplored );
		geometry_msgs::PoseStamped target = transformPixelCoords2PoseStamped( closest );
		std::vector<geometry_msgs::PoseStamped> poseStampedPlan;
		covPath.clear();
		
		ROS_INFO_NAMED("exploration_target_selector","[exploration_target_selector %d] Calculating path to coverage target",__LINE__);
		covPath.push_back( rpose );
		if ( pandoraPathPlanner.makePlan(start, target, poseStampedPlan) ){
			for (unsigned int i=0; i<poseStampedPlan.size(); i++){
				covPath.push_back( transformPoseStamped2PixelCoords(poseStampedPlan[i]) );
			}
			covPath.push_back( closest );
		}
		else{
			ROS_ERROR_NAMED("exploration_target_selector","[exploration_target_selector %d] Did not find path for coverage",__LINE__);
			covPath.clear();
		}
	}
	
	double goalDist=calcPathLength(goal);
	double covDist=calcPathLength(covPath);
	
	ROS_INFO_NAMED("exploration_target_selector","[exploration_target_selector %d] Size of goal path is : %d",__LINE__,(int)goal.size());
	ROS_INFO_NAMED("exploration_target_selector","[exploration_target_selector %d] Size of coverage path is : %d",__LINE__,(int)covPath.size());
	
	//float angleCov=calcAngleOfGoal(*closestCoverageUnexplored);
	//~ if(goal.size()==0 || (goalDist>2.5*covDist && covPath.size()!=0 && closestCoverageUnexplored.computeDistanceFrom(currPos)>50)){
	if(goal.size()==0){
		ROS_ERROR("No node selected from exploration target selector");
		goal.swap(covPath);
		ROS_INFO_NAMED("exploration_target_selector","[exploration_target_selector %d] Coverage preffered ",__LINE__);
		//TODO: clear nodes paths
		*target = closestCoverageUnexplored;
		
	}
	else{
		
		ROS_INFO_NAMED("exploration_target_selector","[exploration_target_selector %d] Voronoi node preffered ",__LINE__);
		//TODO: clear nodes paths
		*target = voronoiNodes.findCoordsFromId(currId);
	}
	
}


void ExplorationTargetSelector::selectTargetWithGaussianExclusion(PixelCoords* target){
	
	
	resetTargets();
	
	int currId;
	unsigned int currX;
	unsigned int currY;
	PixelCoords currPos;
	PixelCoords rpose;

	currX=mapAttributes.robotPose.dx+START_X;
	currY=mapAttributes.robotPose.dy+START_Y;
	currPos=PixelCoords(currX, currY);
	rpose=PixelCoords(mapAttributes.robotPose.dx+START_X,mapAttributes.robotPose.dy+START_Y);
	//~ #ifdef FLOW
		//~ ROS_INFO("[Target Selector %d] mapAttributes->robotPose.dx=%d mapAttributes->robotPose.dy=%d",__LINE__ , mapAttributes.robotPose.dx, mapAttributes.robotPose.dy);
		//~ ROS_INFO("[Target Selector %d] mapAttributes->prevxMax=%d mapAttributes->prevxMin=%d mapAttributes->prevyMax=%d mapAttributes->prevyMin=%d",__LINE__ , mapAttributes.prevxMax,mapAttributes.prevxMin,mapAttributes.prevyMax,mapAttributes.prevyMin);
	//~ #endif

	ROS_INFO_NAMED("exploration_target_selector","[exploration_target_selector %d] Computing voronoi nodes",__LINE__);
	voronoiNodes.createVoronodes();
	
	PixelCoords closestCoverageUnexplored;
	
	ROS_INFO_NAMED("exploration_target_selector","[exploration_target_selector %d] Requesting coverage target from closest_unexplored_target_selector",__LINE__);
	closestUnexploredTargetSelector.selectTarget(&closestCoverageUnexplored);
	closest = closestCoverageUnexplored;
	
	if(closestCoverageUnexplored.getXCoord()==-1 || mapAttributes.map[closestCoverageUnexplored.getXCoord()][closestCoverageUnexplored.getYCoord()]<=127 ){
		
		ROS_ERROR_NAMED("exploration_target_selector","[exploration_target_selector %d] Invalid closestCoverageUnexplored-Cant go with coverage",__LINE__);
	}
	else{
		
		Node coverageNode( closestCoverageUnexplored, voronoiNodes.nodes.size() );
		voronoiNodes.nodes.push_back(coverageNode);
		
	}
	
	
	for(unsigned int i=0;i<voronoiNodes.nodes.size();i++)
		nodesQueue.insert(voronoiNodes.nodes[i].ID);
	
	if(nodesQueue.size()!=0){
		
		
		bool calculateWeights=true;
		float maxWeight=-1,tempWeight;
		int best=-1;
		bool done=false;
		//~ int currId;
		while(!done){
			currX=mapAttributes.robotPose.dx+START_X;
			currY=mapAttributes.robotPose.dy+START_Y;
			currPos=PixelCoords(currX, currY);
			
			if(nodesQueue.size()==0) 
				break;
			
			done=true;
			
			ROS_INFO_NAMED("exploration_target_selector","[exploration_target_selector %d] Selecting node",__LINE__);
			
			if (calculateWeights){
				calculateWeights=false;
				currId=selectNode(nodesQueue);
			}
			else{
				ROS_INFO_NAMED("exploration_target_selector","[exploration_target_selector %d] Using Pre-Calculated Weights",__LINE__);
				maxWeight=-1;
				best=-1;
				for (std::set<unsigned int>::iterator p=nodesQueue.begin();p!=nodesQueue.end();p++){
					tempWeight=calcNodesWeight(*p);
					if(tempWeight>maxWeight){
						maxWeight=tempWeight;
						best=*p;
					}
				}
				currId = best;
			}
			
			//TODO: check if it returns -1 (probably not)
			
			nodesQueue.erase(currId);
			
			
			if(coverage.coverage[voronoiNodes.nodes[currId].p.getXCoord()][voronoiNodes.nodes[currId].p.getYCoord()]>COVERAGE_LIMITS){
				ROS_INFO_NAMED("exploration_target_selector","[exploration_target_selector %d] Node is eliminated by coverage",__LINE__);
				done=false; //break;
			}
			else{
				goal = voronoiNodes.nodes[currId].path;
			}
		
		}
	}
	
	//~ double goalDist=calcPathLength(goal);
	//~ double covDist=calcPathLength(covPath);
	//~ 
	//~ ROS_INFO_NAMED("exploration_target_selector","[exploration_target_selector %d] Size of goal path is : %d",__LINE__,(int)goal.size());
	//~ ROS_INFO_NAMED("exploration_target_selector","[exploration_target_selector %d] Size of coverage path is : %d",__LINE__,(int)covPath.size());
	//~ 
	//~ //float angleCov=calcAngleOfGoal(*closestCoverageUnexplored);
	//~ if(goal.size()==0 || (goalDist>2.5*covDist && covPath.size()!=0 && closestCoverageUnexplored.computeDistanceFrom(currPos)>12)){
		//~ goal.swap(covPath);
		//~ ROS_INFO_NAMED("exploration_target_selector","[exploration_target_selector %d] Coverage preffered ",__LINE__);
		//~ //TODO: clear nodes paths
		//~ *target = closestCoverageUnexplored;
		//~ 
	//~ }
	//~ else{
		//~ 
		//~ ROS_INFO_NAMED("exploration_target_selector","[exploration_target_selector %d] Voronoi node preffered ",__LINE__);
		//~ //TODO: clear nodes paths
		*target = voronoiNodes.findCoordsFromId(currId);
		prevTarget = *target;
	//~ }
	
}


void ExplorationTargetSelector::selectTargetWithGaussianExclusion_newWeights(PixelCoords* target){
	
	
	resetTargets();
	
	int currId;
	unsigned int currX;
	unsigned int currY;
	PixelCoords currPos;
	PixelCoords rpose;

	currX=mapAttributes.robotPose.dx+START_X;
	currY=mapAttributes.robotPose.dy+START_Y;
	currPos=PixelCoords(currX, currY);
	rpose=PixelCoords(mapAttributes.robotPose.dx+START_X,mapAttributes.robotPose.dy+START_Y);
	//~ #ifdef FLOW
		//~ ROS_INFO("[Target Selector %d] mapAttributes->robotPose.dx=%d mapAttributes->robotPose.dy=%d",__LINE__ , mapAttributes.robotPose.dx, mapAttributes.robotPose.dy);
		//~ ROS_INFO("[Target Selector %d] mapAttributes->prevxMax=%d mapAttributes->prevxMin=%d mapAttributes->prevyMax=%d mapAttributes->prevyMin=%d",__LINE__ , mapAttributes.prevxMax,mapAttributes.prevxMin,mapAttributes.prevyMax,mapAttributes.prevyMin);
	//~ #endif

	ROS_INFO_NAMED("exploration_target_selector","[exploration_target_selector %d] Computing voronoi nodes",__LINE__);
	voronoiNodes.createVoronodes();
	
	PixelCoords closestCoverageUnexplored;
	
	ROS_INFO_NAMED("exploration_target_selector","[exploration_target_selector %d] Requesting coverage target from closest_unexplored_target_selector",__LINE__);
	closestUnexploredTargetSelector.selectTarget(&closestCoverageUnexplored);
	closest = closestCoverageUnexplored;
	
	if(closestCoverageUnexplored.getXCoord()==-1 || mapAttributes.map[closestCoverageUnexplored.getXCoord()][closestCoverageUnexplored.getYCoord()]<=127 ){
		
		ROS_ERROR_NAMED("exploration_target_selector","[exploration_target_selector %d] Invalid closestCoverageUnexplored-Cant go with coverage",__LINE__);
	}
	else{
		
		Node coverageNode( closestCoverageUnexplored, voronoiNodes.nodes.size() );
		voronoiNodes.nodes.push_back(coverageNode);
		
	}
	
	
	for(unsigned int i=0;i<voronoiNodes.nodes.size();i++)
		nodesQueue.insert(voronoiNodes.nodes[i].ID);
	
	if(nodesQueue.size()!=0){
		
		
		bool calculateWeights=true;
		float maxWeight=100000,tempWeight;
		int best=-1;
		bool done=false;
		//~ int currId;
		while(!done){
			currX=mapAttributes.robotPose.dx+START_X;
			currY=mapAttributes.robotPose.dy+START_Y;
			currPos=PixelCoords(currX, currY);
			
			if(nodesQueue.size()==0) 
				break;
			
			done=true;
			
			ROS_INFO_NAMED("exploration_target_selector","[exploration_target_selector %d] Selecting node",__LINE__);
			
			if (calculateWeights){
				calculateWeights=false;
				currId=selectNode_newWeights(nodesQueue);
			}
			else{
				ROS_INFO_NAMED("exploration_target_selector","[exploration_target_selector %d] Using Pre-Calculated Weights",__LINE__);
				maxWeight=100000;
				best=-1;
				for (std::set<unsigned int>::iterator p=nodesQueue.begin();p!=nodesQueue.end();p++){
					tempWeight=calcNodesWeight_newWeights(*p);
					if(tempWeight<maxWeight){
						maxWeight=tempWeight;
						best=*p;
					}
				}
				currId = best;
			}
			
			//TODO: check if it returns -1 (probably not)
			
			nodesQueue.erase(currId);
			
			
			if(coverage.coverage[voronoiNodes.nodes[currId].p.getXCoord()][voronoiNodes.nodes[currId].p.getYCoord()]>COVERAGE_LIMITS){
				ROS_INFO_NAMED("exploration_target_selector","[exploration_target_selector %d] Node is eliminated by coverage",__LINE__);
				done=false; //break;
			}
			else{
				goal = voronoiNodes.nodes[currId].path;
			}
		
		}
	}
	
	//~ double goalDist=calcPathLength(goal);
	//~ double covDist=calcPathLength(covPath);
	//~ 
	//~ ROS_INFO_NAMED("exploration_target_selector","[exploration_target_selector %d] Size of goal path is : %d",__LINE__,(int)goal.size());
	//~ ROS_INFO_NAMED("exploration_target_selector","[exploration_target_selector %d] Size of coverage path is : %d",__LINE__,(int)covPath.size());
	//~ 
	//~ //float angleCov=calcAngleOfGoal(*closestCoverageUnexplored);
	//~ if(goal.size()==0 || (goalDist>2.5*covDist && covPath.size()!=0 && closestCoverageUnexplored.computeDistanceFrom(currPos)>12)){
		//~ goal.swap(covPath);
		//~ ROS_INFO_NAMED("exploration_target_selector","[exploration_target_selector %d] Coverage preffered ",__LINE__);
		//~ //TODO: clear nodes paths
		//~ *target = closestCoverageUnexplored;
		//~ 
	//~ }
	//~ else{
		//~ 
		//~ ROS_INFO_NAMED("exploration_target_selector","[exploration_target_selector %d] Voronoi node preffered ",__LINE__);
		//~ //TODO: clear nodes paths
		*target = voronoiNodes.findCoordsFromId(currId);
		prevTarget = *target;
	//~ }
	
}




float ExplorationTargetSelector::calcAngleOfGoal(PixelCoords targetPos){
	float angleToGoal;
	float angleBetween=atan2((targetPos.getYCoord()-(mapAttributes.robotPose.dy+START_Y)),(targetPos.getXCoord()-(mapAttributes.robotPose.dx+START_X)));
	
	if(fabs(mapAttributes.robotPose.theta-angleBetween)<D_PI)
		angleToGoal=fabs(mapAttributes.robotPose.theta-angleBetween);
	else
		angleToGoal=2*D_PI-fabs(mapAttributes.robotPose.theta-angleBetween);
	
	return angleToGoal;
	
}


float ExplorationTargetSelector::calcNodesWeight(int ID){
	
	float weight=0;
	int tempWeight=0;
	
	if(voronoiNodes.nodes[ID].w.dist>=0.6){
		tempWeight|=4;
	}
	else
		tempWeight|=1;
	//~ if(nodes.nodes[ID].w.angle>0.1){
		//~ tempWeight|=2;
	//~ }
	//~ if(voronoiNodes.nodes[ID].w.angle>0.3){
		//~ tempWeight|=2;
	//~ }
	//~ if(voronoiNodes.nodes[ID].w.relativeWallDist>=0.3){
		//~ tempWeight|=1;
		//~ 
	//~ }
	weight=tempWeight;
	
	//~ if(voronoiNodes.nodes[ID].neigh.size()==1) 
		//~ weight*=2;

	weight=weight*(voronoiNodes.nodes[ID].w.dist);
	//~ ROS_INFO("[PLANNER %d] Weight of node %d is %f",__LINE__,ID,weight);
	
	#ifdef EXPLORATION_GAUSSIAN_EXCLUSION
		//~ float SIGMA=2000.0;
		float SIGMA=100.0;
		
		
		//~ float coeff=1.0/(1.0-exp(-pow(voronoiNodes.nodes[ID].p.getXCoord()-prevTarget.getXCoord(),2)/(2.0*SIGMA)-pow(voronoiNodes.nodes[ID].p.getYCoord()-prevTarget.getYCoord(),2)/(2.0*SIGMA)) + 0.2);
		//~ float coeff=(1.0-exp(-pow(voronoiNodes.nodes[ID].p.getXCoord()-prevTarget.getXCoord(),2)/(2.0*SIGMA)-pow(voronoiNodes.nodes[ID].p.getYCoord()-prevTarget.getYCoord(),2)/(2.0*SIGMA)));
		float coeff=pow((1.0-exp(-pow(voronoiNodes.nodes[ID].p.getXCoord()-prevTarget.getXCoord(),2)/(2.0*SIGMA)-pow(voronoiNodes.nodes[ID].p.getYCoord()-prevTarget.getYCoord(),2)/(2.0*SIGMA))),2);
		
		
		
		ROS_DEBUG_NAMED("node_weights","[ExplorationTargetSelector %d] Weight with gaussian of node %d is %f",__LINE__,ID,weight*coeff);
		return weight*coeff;
	#else
		ROS_DEBUG_NAMED("node_weights","[ExplorationTargetSelector %d] Weight of node %d is %f",__LINE__,ID,weight);
		return weight;
	#endif
}


double ExplorationTargetSelector::calcPathLength(std::vector<PixelCoords>& posPath){
	double dist=0;
	if(posPath.size()==0)
		return 0;
	for(int i=0;i<(int)posPath.size()-1;i++){
		dist+=posPath[i].computeDistanceFrom(posPath[i+1]);
	}
	return dist;
}


void ExplorationTargetSelector::performLinearNodesNormalization(std::set<unsigned int> &nodesIDs,VoronoiNodes &nodes){
	//find xmin + xmax
	float xDisMin=2,xDisMax=-1;
	float xAngMin=2,xAngMax=-1;
	float xWallMin=2,xWallMax=-1;
	
	for(std::set<unsigned int>::iterator p=nodesIDs.begin();p!=nodesIDs.end();p++){

		if(nodes.nodes[(*p)].w.dist>xDisMax)
			xDisMax=nodes.nodes[(*p)].w.dist;
		if(nodes.nodes[(*p)].w.dist<xDisMin)
			xDisMin=nodes.nodes[(*p)].w.dist;
			
		if(nodes.nodes[(*p)].w.angle>xAngMax)
			xAngMax=nodes.nodes[(*p)].w.angle;
		if(nodes.nodes[(*p)].w.angle<xAngMin)
			xAngMin=nodes.nodes[(*p)].w.angle;
			
		if(nodes.nodes[(*p)].w.relativeWallDist>xWallMax)
			xWallMax=nodes.nodes[(*p)].w.relativeWallDist;
		if(nodes.nodes[(*p)].w.relativeWallDist<xWallMin)
			xWallMin=nodes.nodes[(*p)].w.relativeWallDist;
	}
	
	for(std::set<unsigned int>::iterator p=nodesIDs.begin();p!=nodesIDs.end();p++){
		
		if(xDisMax!=xDisMin)
			nodes.nodes[(*p)].w.dist=0.1+((nodes.nodes[(*p)].w.dist-xDisMin)/(xDisMax-xDisMin))*0.8;
		
		if(xAngMax!=xAngMin)
			nodes.nodes[(*p)].w.angle=0.1+((nodes.nodes[(*p)].w.angle-xAngMin)/(xAngMax-xAngMin))*0.8;
			
		if(xWallMin!=xWallMax)
			nodes.nodes[*p].w.relativeWallDist=0.1+((nodes.nodes[*p].w.relativeWallDist-xWallMin)/(xWallMax-xWallMin))*0.8;
	}
	
}


void ExplorationTargetSelector::performLinearNodesNormalization_newWeights(std::set<unsigned int> &nodesIDs,VoronoiNodes &nodes){
	//find xmin + xmax
	float xDisMin=1000000000,xDisMax=-1;
	float xAngMin=1000000000,xAngMax=-1;
	float xWallMin=1000000000,xWallMax=-1;
	
	for(std::set<unsigned int>::iterator p=nodesIDs.begin();p!=nodesIDs.end();p++){

		if(nodes.nodes[(*p)].w.dist>xDisMax)
			xDisMax=nodes.nodes[(*p)].w.dist;
		if(nodes.nodes[(*p)].w.dist<xDisMin)
			xDisMin=nodes.nodes[(*p)].w.dist;
			
		if(nodes.nodes[(*p)].w.angle>xAngMax)
			xAngMax=nodes.nodes[(*p)].w.angle;
		if(nodes.nodes[(*p)].w.angle<xAngMin)
			xAngMin=nodes.nodes[(*p)].w.angle;
			
		if(nodes.nodes[(*p)].w.relativeWallDist>xWallMax)
			xWallMax=nodes.nodes[(*p)].w.relativeWallDist;
		if(nodes.nodes[(*p)].w.relativeWallDist<xWallMin)
			xWallMin=nodes.nodes[(*p)].w.relativeWallDist;
	}
	
	for(std::set<unsigned int>::iterator p=nodesIDs.begin();p!=nodesIDs.end();p++){
		
		if(xDisMax!=xDisMin)
			nodes.nodes[(*p)].w.dist=0.1+((nodes.nodes[(*p)].w.dist-xDisMin)/(xDisMax-xDisMin))*0.8;
		
		if(xAngMax!=xAngMin)
			nodes.nodes[(*p)].w.angle=0.1+((nodes.nodes[(*p)].w.angle-xAngMin)/(xAngMax-xAngMin))*0.8;
			
		if(xWallMin!=xWallMax)
			nodes.nodes[*p].w.relativeWallDist=0.1+((nodes.nodes[*p].w.relativeWallDist-xWallMin)/(xWallMax-xWallMin))*0.8;
	}
	
}


void ExplorationTargetSelector::performNodesNormalization(std::set<unsigned int> &nodesIDs,VoronoiNodes &nodes){
	float sumNodesDist=0;
	float sumNodesAngles=0;
	float sumNodesWallDist=0;
	
	for(std::set<unsigned int>::iterator p=nodesIDs.begin();p!=nodesIDs.end();p++){
		sumNodesDist+=nodes.nodes[(*p)].w.dist;
		sumNodesAngles+=nodes.nodes[(*p)].w.angle;
		sumNodesWallDist+=nodes.nodes[*p].w.relativeWallDist;
	}
	for(std::set<unsigned int>::iterator p=nodesIDs.begin();p!=nodesIDs.end();p++){
		
		if(nodes.nodes[(*p)].w.dist!=0)
			nodes.nodes[(*p)].w.dist=(sumNodesDist-nodes.nodes[(*p)].w.dist)/sumNodesDist;
		else
			nodes.nodes[(*p)].w.dist=0.3;
		
		nodes.nodes[(*p)].w.angle=(sumNodesAngles-nodes.nodes[(*p)].w.angle)/sumNodesAngles;
		nodes.nodes[*p].w.relativeWallDist=(sumNodesWallDist-nodes.nodes[*p].w.relativeWallDist)/sumNodesWallDist;
	}
	
}


//~ void ExplorationTargetSelector::calcNodeParameters(std::set<unsigned int> &graph){
void ExplorationTargetSelector::calcNodeParameters(void){
	//~ vector<PixelCoords> tempPath;
	
	ROS_INFO_NAMED("exploration_target_selector","[exploration_target_selector %d] Calculating paths",__LINE__);
	pandoraPathPlanner.getTargetSelectorPaths( &voronoiNodes );
	//TODO: use previously calulated paths!!!!!!!!!!
	
	//~ for(std::set<unsigned int>::iterator p=graph.begin();p!=graph.end();p++){
	ROS_INFO_NAMED("exploration_target_selector","[exploration_target_selector %d] Calculating other params",__LINE__);
	for(unsigned int i=0; i< voronoiNodes.nodes.size(); i++){
		
		//~ #ifdef FLOW 
			//~ ROS_INFO("[PLANNER %d] Insert new node temporarily in partition",__LINE__);
		//~ #endif
		//~ int tempID=incrementalPartitionGraph.insertNodeInPartition(voronoiNodes.nodes[(*p)]);
		//~ #ifdef FLOW 
			//~ ROS_INFO("[PLANNER %d] Running Dikstraj algorithm",__LINE__);
		//~ #endif
		//~ findShortestPath(incrementalPartitionGraph.nodes[0],incrementalPartitionGraph.nodes[tempID],incrementalPartitionGraph,tempPath);
		//~ #ifdef FLOW 
			//~ ROS_INFO("[PLANNER %d] I am done with Dikstraj",__LINE__);
		//~ #endif
		//~ tempPath.push_back(incrementalPartitionGraph.nodes[tempID].p);
		
		if(voronoiNodes.nodes[i].path.size()!=0)
			voronoiNodes.nodes[i].w.dist=calcPathLength(voronoiNodes.nodes[i].path);
			//~ voronoiNodes.nodes[i].w.dist=calcPathLength(tempPath);
		else
			voronoiNodes.nodes[i].w.dist=0;
		
		
		voronoiNodes.nodes[i].w.angle=calcAngleOfGoal(voronoiNodes.nodes[i].p);
		voronoiNodes.nodes[i].w.relativeWallDist=voronoiNodes.voronoi.brushCell[voronoiNodes.nodes[i].p.getXCoord()][voronoiNodes.nodes[i].p.getYCoord()];
		
		//~ partitionNodes.eliminateNode(tempID,true);
		//~ #ifdef FLOW 
			//~ ROS_INFO("[PLANNER %d] Eliminated temp voronode",__LINE__);
		//~ #endif
	}
}



void ExplorationTargetSelector::calcNodeParameters_newWeights(void){
	//~ vector<PixelCoords> tempPath;
	
	ROS_INFO_NAMED("exploration_target_selector","[exploration_target_selector %d] Calculating paths",__LINE__);
	pandoraPathPlanner.getTargetSelectorPaths( &voronoiNodes );
	//TODO: use previously calulated paths!!!!!!!!!!
	
	//~ for(std::set<unsigned int>::iterator p=graph.begin();p!=graph.end();p++){
	ROS_INFO_NAMED("exploration_target_selector","[exploration_target_selector %d] Calculating other params",__LINE__);
	for(unsigned int i=0; i< voronoiNodes.nodes.size(); i++){
		
		//~ #ifdef FLOW 
			//~ ROS_INFO("[PLANNER %d] Insert new node temporarily in partition",__LINE__);
		//~ #endif
		//~ int tempID=incrementalPartitionGraph.insertNodeInPartition(voronoiNodes.nodes[(*p)]);
		//~ #ifdef FLOW 
			//~ ROS_INFO("[PLANNER %d] Running Dikstraj algorithm",__LINE__);
		//~ #endif
		//~ findShortestPath(incrementalPartitionGraph.nodes[0],incrementalPartitionGraph.nodes[tempID],incrementalPartitionGraph,tempPath);
		//~ #ifdef FLOW 
			//~ ROS_INFO("[PLANNER %d] I am done with Dikstraj",__LINE__);
		//~ #endif
		//~ tempPath.push_back(incrementalPartitionGraph.nodes[tempID].p);
		
		if(voronoiNodes.nodes[i].path.size()!=0)
			voronoiNodes.nodes[i].w.dist=calcPathLength(voronoiNodes.nodes[i].path);
			//~ voronoiNodes.nodes[i].w.dist=calcPathLength(tempPath);
		else
			voronoiNodes.nodes[i].w.dist=0;
		
		
		voronoiNodes.nodes[i].w.angle=calcAngleOfGoal(voronoiNodes.nodes[i].p);
		//~ voronoiNodes.nodes[i].w.relativeWallDist=voronoiNodes.voronoi.brushCell[voronoiNodes.nodes[i].p.getXCoord()][voronoiNodes.nodes[i].p.getYCoord()];
		voronoiNodes.nodes[i].w.relativeWallDist=calcEnclosure(voronoiNodes.nodes[i].p);
		
		//~ partitionNodes.eliminateNode(tempID,true);
		//~ #ifdef FLOW 
			//~ ROS_INFO("[PLANNER %d] Eliminated temp voronode",__LINE__);
		//~ #endif
	}
}



void ExplorationTargetSelector::resetTargets(void){
	goal.clear();
	nodesQueue.clear();
	//~ currGoal=0;	
	//~ lockTarget=false;
}


int ExplorationTargetSelector::selectNode(std::set<unsigned int> &graph){
	float maxWeight=-1,tempWeight;
	int best=-1;
	PixelCoords startPos=PixelCoords(mapAttributes.robotPose.dx+START_X,mapAttributes.robotPose.dy+START_Y);
	//~ incrementalPartitionGraph.createIncrementalPartitioningGraph(startPos,startPos);
	
	if(graph.size()==1) {
		
		ROS_INFO_NAMED("exploration_target_selector","[exploration_target_selector %d] Graph has one node",__LINE__);
		return *(graph.begin());
	}
	
	ROS_INFO_NAMED("exploration_target_selector","[exploration_target_selector %d] Calculating nodes parameters",__LINE__);
	//~ calcNodeParameters(graph);
	calcNodeParameters();
	
	//~ ROS_INFO("[PLANNER %d] ----- PRINTING MEASURMENTS -----",__LINE__);
	//~ for (std::set<unsigned int>::iterator p=graph.begin();p!=graph.end();p++){
		//~ ROS_INFO("");
		//~ ROS_INFO("[PLANNER %d]--For Node %d measurements  are : ",__LINE__,(*p));
		//~ ROS_INFO("[PLANNER %d]	Angle = %f ",__LINE__,nodes.nodes[(*p)].w.angle*180/D_PI);
		//~ ROS_INFO("[PLANNER %d]	Distance = %f ",__LINE__,nodes.nodes[(*p)].w.dist);
		//~ ROS_INFO("[PLANNER %d]	WallDistance = %f ",__LINE__,nodes.nodes[(*p)].w.relativeWallDist);
		//~ ROS_INFO("");
		
	//~ }
	
	//~ performNodesNormalization(graph,nodes);
	//~ performLinearNodesNormalization(graph,nodes);
	ROS_INFO_NAMED("exploration_target_selector","[exploration_target_selector %d] Normalizing nodes parameters",__LINE__);
	performNodesNormalization(graph,voronoiNodes);
	ROS_INFO_NAMED("exploration_target_selector","[exploration_target_selector %d] Performing linear normalization",__LINE__);
	performLinearNodesNormalization(graph,voronoiNodes);
	
	//~ ROS_INFO("[PLANNER %d] ----- PRINTING WEIGHTS -----",__LINE__);
	//~ for (std::set<unsigned int>::iterator p=graph.begin();p!=graph.end();p++){
	//~ #ifdef FLOW
		//~ ROS_INFO("[PLANNER %d]--For Node %d measurements  are : ",__LINE__,(*p));
		//~ ROS_INFO("[PLANNER %d]	Angle = %f ",__LINE__,nodes.nodes[(*p)].w.angle);
		//~ ROS_INFO("[PLANNER %d]	Distance = %f ",__LINE__,nodes.nodes[(*p)].w.dist);
		//~ ROS_INFO("[PLANNER %d]	WallDistance = %f ",__LINE__,nodes.nodes[(*p)].w.relativeWallDist);
		//~ ROS_INFO("-");
	//~ #endif
	//~ }
	
	
	for (std::set<unsigned int>::iterator p=graph.begin();p!=graph.end();p++){
		tempWeight=calcNodesWeight(*p);
		if(tempWeight>maxWeight){
			maxWeight=tempWeight;
			best=*p;
		}
	}
	#ifdef FLOW 
		ROS_INFO("[EXPLORATION_TARGET_SELECTOR %d] Found best weight of node and it is : %f",__LINE__,maxWeight);
	#endif
	ROS_INFO_NAMED("exploration_target_selector","[exploration_target_selector %d] Found best weight of node and it is : %f",__LINE__,maxWeight);
	return best;
	
}


int ExplorationTargetSelector::selectNode_newWeights(std::set<unsigned int> &graph){
	float maxWeight=100000,tempWeight;
	int best=-1;
	PixelCoords startPos=PixelCoords(mapAttributes.robotPose.dx+START_X,mapAttributes.robotPose.dy+START_Y);
	//~ incrementalPartitionGraph.createIncrementalPartitioningGraph(startPos,startPos);
	
	if(graph.size()==1) {
		
		ROS_INFO_NAMED("exploration_target_selector","[exploration_target_selector %d] Graph has one node",__LINE__);
		return *(graph.begin());
	}
	
	ROS_INFO_NAMED("exploration_target_selector","[exploration_target_selector %d] Calculating nodes parameters",__LINE__);
	//~ calcNodeParameters(graph);
	calcNodeParameters_newWeights();
	
	//~ ROS_INFO("[PLANNER %d] ----- PRINTING MEASURMENTS -----",__LINE__);
	//~ for (std::set<unsigned int>::iterator p=graph.begin();p!=graph.end();p++){
		//~ ROS_INFO("");
		//~ ROS_INFO("[PLANNER %d]--For Node %d measurements  are : ",__LINE__,(*p));
		//~ ROS_INFO("[PLANNER %d]	Angle = %f ",__LINE__,nodes.nodes[(*p)].w.angle*180/D_PI);
		//~ ROS_INFO("[PLANNER %d]	Distance = %f ",__LINE__,nodes.nodes[(*p)].w.dist);
		//~ ROS_INFO("[PLANNER %d]	WallDistance = %f ",__LINE__,nodes.nodes[(*p)].w.relativeWallDist);
		//~ ROS_INFO("");
		
	//~ }
	
	//~ performNodesNormalization(graph,nodes);
	//~ performLinearNodesNormalization(graph,nodes);
	//~ ROS_INFO_NAMED("exploration_target_selector","[exploration_target_selector %d] Normalizing nodes parameters",__LINE__);
	//~ performNodesNormalization(graph,voronoiNodes);
	ROS_INFO_NAMED("exploration_target_selector","[exploration_target_selector %d] Performing linear normalization",__LINE__);
	performLinearNodesNormalization_newWeights(graph,voronoiNodes);
	
	//~ ROS_INFO("[PLANNER %d] ----- PRINTING WEIGHTS -----",__LINE__);
	//~ for (std::set<unsigned int>::iterator p=graph.begin();p!=graph.end();p++){
	//~ #ifdef FLOW
		//~ ROS_INFO("[PLANNER %d]--For Node %d measurements  are : ",__LINE__,(*p));
		//~ ROS_INFO("[PLANNER %d]	Angle = %f ",__LINE__,nodes.nodes[(*p)].w.angle);
		//~ ROS_INFO("[PLANNER %d]	Distance = %f ",__LINE__,nodes.nodes[(*p)].w.dist);
		//~ ROS_INFO("[PLANNER %d]	WallDistance = %f ",__LINE__,nodes.nodes[(*p)].w.relativeWallDist);
		//~ ROS_INFO("-");
	//~ #endif
	//~ }
	
	
	for (std::set<unsigned int>::iterator p=graph.begin();p!=graph.end();p++){
		tempWeight=calcNodesWeight_newWeights(*p);
		if(tempWeight<maxWeight){
			maxWeight=tempWeight;
			best=*p;
		}
	}
	#ifdef FLOW 
		ROS_INFO("[EXPLORATION_TARGET_SELECTOR %d] Found best weight of node and it is : %f",__LINE__,maxWeight);
	#endif
	ROS_INFO_NAMED("exploration_target_selector","[exploration_target_selector %d] Found best weight of node and it is : %f",__LINE__,maxWeight);
	return best;
	
}


float ExplorationTargetSelector::calcNodesWeight_newWeights(int ID){
	
	float weight=0;
	int tempWeight=0;
	
	if(voronoiNodes.nodes[ID].w.dist>=0.6){
		tempWeight|=4;
	}
	//~ if(nodes.nodes[ID].w.angle>0.1){
		//~ tempWeight|=2;
	//~ }
	//~ if(voronoiNodes.nodes[ID].w.angle>0.3){
		//~ tempWeight|=2;
	//~ }
	//~ if(voronoiNodes.nodes[ID].w.relativeWallDist>=0.3){
		//~ tempWeight|=1;
		//~ 
	//~ }

	//~ switch (tempWeight){
		//~ case 1:
			//~ weight=1;
			//~ break;
		//~ case 2:
			//~ weight=2;
			//~ break;
		//~ case 3:
			//~ weight=3;
			//~ break;
		//~ case 4:
			//~ weight=4;
			//~ break;
		//~ case 5:
			//~ weight=5;
			//~ break;
		//~ case 6:
			//~ weight=6;
			//~ break;
		//~ case 7:
			//~ weight=7;
			//~ break;
		//~ default : 
			//~ weight=0;
	//~ }
	weight = tempWeight;
	
	
	//~ if(voronoiNodes.nodes[ID].neigh.size()==1) 
		//~ weight*=2;

	//~ weight = (1 + weight) * ((4* voronoiNodes.nodes[ID].w.relativeWallDist) + (2 *(voronoiNodes.nodes[ID].w.dist)) + voronoiNodes.nodes[ID].w.angle);
	
	weight = (1 + weight) * ((2 *(voronoiNodes.nodes[ID].w.dist)));
	//~ ROS_INFO("[PLANNER %d] Weight of node %d is %f",__LINE__,ID,weight);
	
	#ifdef EXPLORATION_GAUSSIAN_EXCLUSION_NEW_WEIGHTS
		//~ float SIGMA=2000.0;
		float SIGMA=100.0;
		
		
		float coeff=1.0/(1.01-exp(-pow(voronoiNodes.nodes[ID].p.getXCoord()-prevTarget.getXCoord(),2)/(2.0*SIGMA)-pow(voronoiNodes.nodes[ID].p.getYCoord()-prevTarget.getYCoord(),2)/(2.0*SIGMA)));
		//~ float coeff=(1.0-exp(-pow(voronoiNodes.nodes[ID].p.getXCoord()-prevTarget.getXCoord(),2)/(2.0*SIGMA)-pow(voronoiNodes.nodes[ID].p.getYCoord()-prevTarget.getYCoord(),2)/(2.0*SIGMA)));
		//~ float coeff=pow((1.0-exp(-pow(voronoiNodes.nodes[ID].p.getXCoord()-prevTarget.getXCoord(),2)/(2.0*SIGMA)-pow(voronoiNodes.nodes[ID].p.getYCoord()-prevTarget.getYCoord(),2)/(2.0*SIGMA))),2);
		
		
		
		ROS_DEBUG_NAMED("node_weights","[ExplorationTargetSelector %d] Weight with gaussian of node %d is %f",__LINE__,ID,weight*coeff);
		return weight*coeff;
	#else
		ROS_DEBUG_NAMED("node_weights","[ExplorationTargetSelector %d] Weight of node %d is %f",__LINE__,ID,weight);
		return weight;
	#endif
}



float ExplorationTargetSelector::calcEnclosure(PixelCoords p){
	float sum=0;
	float ang=0;
	float x,y;
	float D;
	//~ cout2<<"\n";
	float xinit=p.getXCoord(),yinit=p.getYCoord();
	for(unsigned int i=0;i<8;i++){
		ang=2.0*D_PI/8.0*i;
		D=0;
		for(D=0;D<200;D+=1){
			x=xinit+D*cos(ang);
			y=yinit+D*sin(ang);
			if(mapAttributes.map[(int)x][(int)y]<127)
				break;
			else if(mapAttributes.map[(int)x][(int)y]==127){
				D=200;
				break;
			}
		}
		//~ cout2<<D<<" at ang= "<<ang/D_PI*180.0<<"\n";
		sum+=D;
	}
	return sum/8.0;
}
