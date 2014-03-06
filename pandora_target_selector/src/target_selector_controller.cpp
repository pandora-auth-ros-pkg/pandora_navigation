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

#include "pandora_target_selector/target_selector_controller.h"

TargetSelectorController::TargetSelectorController(void): selectTargetActionServer_(nh_, "select_target", boost::bind(&TargetSelectorController::selectTargetActionCallBack, this, _1), false),
										mapAttributes( _mapUpdater.getMapAttributes() ),
										coverage( mapAttributes.getHeight(),mapAttributes.getWidth() ),
										//~ closestUnexploredTargetSelector( mapAttributes, coverage )
										closestUnexploredTargetSelector( _mapUpdater.getMapAttributes(), coverage ),
										explorationTargetSelector( _mapUpdater.getMapAttributes(), coverage ),
										victimTargetSelector( _mapUpdater.getMapAttributes(), coverage )
{

	

	_mapService= nh_.advertiseService("NavigationMapSrv/targetSelectorController",&TargetSelectorController::sendMap,this);

	_flushService = nh_.advertiseService("/navigation/flush_coverage", 
				&TargetSelectorController::flushCoverage, this);
	
	//~ robotPoseSubscriber= nh_.subscribe("/slam/robotPose", 1, &TargetSelectorController::patchCoverage,this);
	selectTargetActionServer_.start();
	_mapUpdater.startSubscriber();
	
	
	
	geotiffSrv=nh_.advertiseService("/navigation/geotiffSrv",&TargetSelectorController::geotiffSericeCb,this);
	
	_coverageCallbackTimer = nh_.createTimer(ros::Duration(1),&TargetSelectorController::coverageTimerCallback,this,false,false);
	_coveragePatchingTimer = nh_.createTimer(ros::Duration(1),&TargetSelectorController::patchCoverage,this,false,false);
	
	_occupancyGridPublisher = nh_.advertise<nav_msgs::OccupancyGrid>("/navigation/coverage",1000);
	
	while(!_mapUpdater.slamInitialized && ros::ok()){
		ros::Duration(1).sleep();
		ros::spinOnce();
	}
	_coverageCallbackTimer.start();
	_coveragePatchingTimer.start();
}

void TargetSelectorController::makeDummyCoveragePatch(unsigned char **map,unsigned char **coverage,int x, int y,int d){
	float angle=0;
	int tx,ty;
	while (angle<6.28){
		for(unsigned int i=0;i<d;i++){
			tx=x+i*cos(angle);
			ty=y+i*sin(angle);
			if((coverage[tx][ty]+3)<=255 && map[tx][ty]>127)
				coverage[tx][ty]+=3;
		}
		angle+=0.0004;
	}
}

void TargetSelectorController::selectTargetActionCallBack(const target_selector_communications::SelectTargetGoalConstPtr &goal){
	ROS_INFO_NAMED("target_selector_controller","[target_selector_controller %d] Finding next target",__LINE__);
	//~ _mapUpdater.startSubscriber();
	//~ if (temp){
		//~ std::cout<<"fixing coverage patch\n";
		//~ //-----------------------------------
		//~ makeDummyCoveragePatch((_mapUpdater.getMapAttributes()).map,coverage.coverage,2046,2046,rand()%2000);
		//~ for(unsigned int i=0;i<50;i++){
			//~ int x=0,y=0;
			//~ while((_mapUpdater.getMapAttributes()).map[x][y]<130){
				//~ x=(_mapUpdater.getMapAttributes()).prevxMin+rand()%((_mapUpdater.getMapAttributes()).prevxMax-(_mapUpdater.getMapAttributes()).prevxMin);
				//~ y=(_mapUpdater.getMapAttributes()).prevyMin+rand()%((_mapUpdater.getMapAttributes()).prevyMax-(_mapUpdater.getMapAttributes()).prevyMin);
			//~ }
			//~ makeDummyCoveragePatch((_mapUpdater.getMapAttributes()).map,coverage.coverage,x,y,rand()%200);
		//~ }
		//~ //--------------------------------
		//~ temp =false;
		//~ std::cout<<"finished fixing coverage patch\n";
	//~ }
	
	
	
	target_selector_communications::SelectTargetResult result; 
	
	PixelCoords target;
	geometry_msgs::PoseStamped poseStampedTarget;
	switch (goal->targetType){
		case target_selector_communications::SelectTargetGoal::TYPE_VICTIM :{
			ROS_INFO_NAMED("target_selector_controller","[target_selector_controller %d] Requesting victim target",__LINE__);
			bool hasVictims;
			
			hasVictims = victimTargetSelector.selectTarget(&poseStampedTarget);
			
			if ( hasVictims ){
				result.target_pose = poseStampedTarget;
				//~ result.target_pose = transformPixelCoords2PoseStamped(target);
				ROS_INFO_NAMED("target_selector_controller","[target_selector_controller %d] Victim target found",__LINE__);
				selectTargetActionServer_.setSucceeded(result);
			}
			else{
				ROS_ERROR_NAMED("target_selector_controller","[target_selector_controller %d] Victim target NOT found",__LINE__);
				selectTargetActionServer_.setAborted(result);
			}
			
		}
		case target_selector_communications::SelectTargetGoal::TYPE_EXPLORATION :{
			ROS_INFO_NAMED("target_selector_controller","[target_selector_controller %d] Requesting exploration target",__LINE__);
			
			
			if ( checkPreempt() ){
				ROS_ERROR_NAMED("target_selector_controller", "[target_selector_controller %d]: Selecting target preempted.", __LINE__);
				selectTargetActionServer_.setPreempted(result);
				//~ return false;
				return;
			}
			
			//---select target
			#ifdef CLOSEST_UNEXPLORED
				closestUnexploredTargetSelector.selectTarget(&target);
			#elif defined SIMPLE_EXPLORATION
				explorationTargetSelector.selectTarget(&target);
			#elif defined EXPLORATION_GAUSSIAN_EXCLUSION
				explorationTargetSelector.selectTargetWithGaussianExclusion(&target);
			#elif defined EXPLORATION_GAUSSIAN_EXCLUSION_NEW_WEIGHTS
				explorationTargetSelector.selectTargetWithGaussianExclusion_newWeights(&target);
			#else
				selectTargetActionServer_.setAborted(result);
			#endif
			
			result.target_pose = transformPixelCoords2PoseStamped(target);
			//---transform target pose to meters
			result.target_pose.pose.position.x = (result.target_pose.pose.position.x - START_X) * 0.02;
			result.target_pose.pose.position.y = (result.target_pose.pose.position.y - START_Y) * 0.02;
			
			if ( checkPreempt() ){
				ROS_ERROR_NAMED("target_selector_controller", "[target_selector_controller %d]: Selecting target preempted.", __LINE__);
				selectTargetActionServer_.setPreempted(result);
				//~ return false;
				return;
			}
			
			
			ROS_INFO_NAMED("target_selector_controller","[target_selector_controller %d] Target found",__LINE__);
			selectTargetActionServer_.setSucceeded(result);
		}
		
	}
}


void TargetSelectorController::patchCoverage(const ros::TimerEvent&){
	coverage.patchMapAt((int)(mapAttributes.robotPose.dx+MAP_SIZE/2), (int)(mapAttributes.robotPose.dy+MAP_SIZE/2), mapAttributes.robotPose.theta, mapAttributes.map, 0, 0, 0);
}



bool TargetSelectorController::checkPreempt(){
		if(selectTargetActionServer_.isPreemptRequested()==true || !ros::ok()){
			return true;
		}
		return false;
}

geometry_msgs::PoseStamped TargetSelectorController::transformPixelCoords2PoseStamped(PixelCoords pixelCoords){
	
	float x = pixelCoords.getXCoord();
	float y = pixelCoords.getYCoord();
	
	geometry_msgs::PoseStamped poseStamped;
	
	poseStamped.pose.position.x = x;
	poseStamped.pose.position.y = y;
	
	return poseStamped;
}

bool TargetSelectorController::sendMap(pandora_navigation_communications::navigationMapSrv::Request &req,pandora_navigation_communications::navigationMapSrv::Response &res){

	if(!_mapUpdater.slamInitialized) return false;

	int prevxMax = mapAttributes.prevxMax;
	int prevyMax = mapAttributes.prevyMax;
	int prevxMin = mapAttributes.prevxMin;
	int prevyMin = mapAttributes.prevyMin;
	res.prevxmin=prevxMin;
	res.prevymin=prevyMin;
	res.prevxmax=prevxMax;
	res.prevymax=prevyMax;
	
	//--------------------- voronoi nodes ---------------------// (from targetSelector)
	
	res.prevxmin=prevxMin;
	res.prevymin=prevyMin;
	res.prevxmax=prevxMax;
	res.prevymax=prevyMax;
	
	res.voronodesx=std::vector<int>(explorationTargetSelector.voronoiNodes.nodes.size());
	res.voronodesy=std::vector<int>(explorationTargetSelector.voronoiNodes.nodes.size());
	for(unsigned int i=0;i<explorationTargetSelector.voronoiNodes.nodes.size();i++){
		res.voronodesx[i]=explorationTargetSelector.voronoiNodes.nodes[i].p.getXCoord()-prevxMin;
		res.voronodesy[i]=explorationTargetSelector.voronoiNodes.nodes[i].p.getYCoord()-prevyMin;
	}
	
	for(unsigned int i=0;i<explorationTargetSelector.voronoiNodes.nodes.size();i++)
		for(unsigned int j=0;j<explorationTargetSelector.voronoiNodes.nodes[i].neighID.size();j++){
			res.neighborsFirst.push_back(explorationTargetSelector.voronoiNodes.nodes[i].ID);
			res.neighborsLast.push_back(explorationTargetSelector.voronoiNodes.nodes[i].neighID[j]);
		}


	res.voronoi=std::vector<unsigned char>((prevxMax-prevxMin)*(prevyMax-prevyMin));
	for(int i=0;i<(prevxMax-prevxMin);i++)
		for(int j=0;j<(prevyMax-prevyMin);j++)
			res.voronoi[i*(prevyMax-prevyMin)+j]=explorationTargetSelector.voronoiNodes.voronoi.voronoi[i+prevxMin][j+prevyMin];
	
	return true;
}

bool TargetSelectorController::flushCoverage(std_srvs::Empty::Request& rq,std_srvs::Empty::Response &rs){
	
	coverage.flush();
	
	return true;
	
}


bool TargetSelectorController::geotiffSericeCb(pandora_navigation_communications::NavigationGeotiffSrv::Request &req,pandora_navigation_communications::NavigationGeotiffSrv::Response &res){
	//map
	
	int prevxMax = mapAttributes.prevxMax;
	int prevyMax = mapAttributes.prevyMax;
	int prevxMin = mapAttributes.prevxMin;
	int prevyMin = mapAttributes.prevyMin;
	
	res.xsize=prevxMax-prevxMin;
	res.ysize=prevyMax-prevyMin;
	res.map=std::vector<unsigned char>((prevxMax-prevxMin)*(prevyMax-prevyMin));
	for(int i=0;i<(prevxMax-prevxMin);i++)
		for(int j=0;j<(prevyMax-prevyMin);j++)
			res.map[i*(prevyMax-prevyMin)+j]=mapAttributes.map[i+prevxMin][j+prevyMin];	
	
	
	//coverage
	res.coverage=std::vector<unsigned char>((mapAttributes.width)*(mapAttributes.height));
	for(int i=0;i<(mapAttributes.prevxMax-mapAttributes.prevxMin);i++)
		for(int j=0;j<(mapAttributes.prevyMax-mapAttributes.prevyMin);j++)
			res.coverage[i*(mapAttributes.prevyMax-mapAttributes.prevyMin)+j]=
				coverage.coverage[i+mapAttributes.prevxMin][j+mapAttributes.prevyMin];

	//trajectory
	res.rposesx=std::vector<int>(mapAttributes.robotTrajectory.size());
	res.rposesy=std::vector<int>(mapAttributes.robotTrajectory.size());
	for(unsigned int i=0;i<mapAttributes.robotTrajectory.size();i++){
		res.rposesx[i]=mapAttributes.robotTrajectory[i].dx +  START_X - prevxMin;
		res.rposesy[i]=mapAttributes.robotTrajectory[i].dy +  START_Y - prevyMin;
	}
	
	return true;
}

void TargetSelectorController::coverageTimerCallback(const ros::TimerEvent&){
	nav_msgs::OccupancyGrid grid;
	
	int prevxMax = mapAttributes.prevxMax;
	int prevyMax = mapAttributes.prevyMax;
	int prevxMin = mapAttributes.prevxMin;
	int prevyMin = mapAttributes.prevyMin;
    
    grid.header.stamp = ros::Time::now(); 
    grid.header.frame_id = "map"; 
    
    int width=prevxMax-prevxMin;
    int height=prevyMax-prevyMin;
    
    grid.info.resolution = 0.02;
    grid.info.width = width;
    grid.info.height = height;

    grid.info.origin.position.x = - (START_X - prevxMin)*0.02 ;
    grid.info.origin.position.y = - (START_Y - prevyMin)*0.02 ;
    
    grid.data.resize(width*height) ;
    for(int i=0;i<(width);i++){
		for(int j=0;j<(height);j++){
			grid.data[j*width+i]= (int) (coverage.coverage[i+prevxMin][j+prevyMin]/255.0*100.0);
		}
    }
    
    _occupancyGridPublisher.publish(grid);
}

