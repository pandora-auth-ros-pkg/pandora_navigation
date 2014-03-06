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

#include "pandora_target_selector/target_selector.h"

//TODO: edit this. needs robot pose and coverage
//void TargetSelector::slamMapCallback(const actionlib::SimpleClientGoalState& state, const slam_communications::slamMapResultConstPtr& result){

	//int width=result->xsize;
	//int height=result->ysize;
	
	//prevxMin=result->xmin;
	//prevxMax=result->xmax;
	//prevyMin=result->ymin;
	//prevyMax=result->ymax;
	
	//for(int i=0;i<width;i++){
		//for(int j=0;j<height;j++){
			//map[i+prevxMin][j+prevyMin]=result->slamMap[i*height+j];
		//}
	//}
//}

//~ TargetSelector::TargetSelector():  mapAttributes(_mapUpdater.getMapAttributes()){
TargetSelector::TargetSelector(MapAttributes& mapAttr, Coverage& cov): mapAttributes(mapAttr), coverage(cov){
	
	//~ mapAttributes(mapAttr);
	//~ TODO: initialize map and coverage with predefined values
	
	//~ mapAttributes = & (_mapUpdater.getMapAttributes() );
	//~ _mapUpdater.startSubscriber();

	//~ robotPoseSubscriber= _nh.subscribe("/slam/robotPose", 1, &TargetSelector::patchCoverage,this);

}


//~ void TargetSelector::patchCoverage(const slam_communications::robotPoseMsg& msg){
	//~ 
	//~ coverage.patchMapAt((int)(msg.x+MAP_SIZE/2), (int)(msg.y+MAP_SIZE/2), msg.theta, mapAttributes.map, 0, 0, 0);
//~ 
	//~ ROS_DEBUG("[TARGET_SELECTOR %d] Coverage patched at %d,%d,%f",__LINE__,msg.x, msg.y, msg.theta);
//~ 
//~ }


PixelCoords TargetSelector::transformPoseStamped2PixelCoords(geometry_msgs::PoseStamped poseStamped){
	
	int x = poseStamped.pose.position.x;
	int y = poseStamped.pose.position.y;
	
	return PixelCoords(x,y) ;
	
}



geometry_msgs::PoseStamped TargetSelector::transformPixelCoords2PoseStamped(PixelCoords pixelCoords){
	
	float x = pixelCoords.getXCoord();
	float y = pixelCoords.getYCoord();
	
	geometry_msgs::PoseStamped poseStamped;
	
	poseStamped.pose.position.x = x;
	poseStamped.pose.position.y = y;
	
	return poseStamped;
	
	
}
