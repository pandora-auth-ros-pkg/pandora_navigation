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

#include "pandora_target_selector/victim_target_selector.h"



VictimTargetSelector::VictimTargetSelector(MapAttributes& mapAttr, Coverage& cov):
								TargetSelector(mapAttr, cov),
								_getVictimsActionClient("/data_fusion/alert_handler/get_victims",true)
{
	_selectedTargetIndexPublisher = _nh.advertise<std_msgs::Int16>("/target_selector/selected_victim",100,true);
}

void VictimTargetSelector::selectTarget(PixelCoords* target){
	
	std::vector<geometry_msgs::PoseStamped> victimsVector;
	victimsVector.clear();
	
	data_fusion_communications::GetVictimsGoal getVictimsGoal;
	
	_getVictimsActionClient.waitForServer();
	_getVictimsActionClient.sendGoal(getVictimsGoal);
	_getVictimsActionClient.waitForResult();
	
	
	for (unsigned int i=0; i<_getVictimsActionClient.getResult()->victimsArray.size(); i++){
		
		victimsVector.push_back(_getVictimsActionClient.getResult()->victimsArray[i].victimPose);
		
	}
	
	*target = transformPoseStamped2PixelCoords( chooseVictim(victimsVector) );
	
	
	
}


//~ void VictimTargetSelector::selectTarget(geometry_msgs::PoseStamped* target){
bool VictimTargetSelector::selectTarget(geometry_msgs::PoseStamped* target){
	
	std::vector<geometry_msgs::PoseStamped> victimsVector;
	victimsVector.clear();
	
	data_fusion_communications::GetVictimsGoal getVictimsGoal;
	
	ROS_INFO_NAMED("victim_target_selector","[victim_target_selector %d] Requesting victims",__LINE__);
	
	_getVictimsActionClient.waitForServer();
	_getVictimsActionClient.sendGoal(getVictimsGoal);
	_getVictimsActionClient.waitForResult();
	
	if (_getVictimsActionClient.getResult()->victimsArray.size()==0 ){
		ROS_INFO_NAMED("victim_target_selector","[victim_target_selector %d] No victims",__LINE__);
		return false;
	}
	
	
	
	ROS_INFO_NAMED("victim_target_selector","[victim_target_selector %d] Requesting victims",__LINE__);
	for (unsigned int i=0; i<_getVictimsActionClient.getResult()->victimsArray.size(); i++){
		
		victimsVector.push_back(_getVictimsActionClient.getResult()->victimsArray[i].victimPose);
		
	}
	
	ROS_INFO_NAMED("victim_target_selector","[victim_target_selector %d] Selecting victim",__LINE__);
	//~ *target = transformPoseStamped2PixelCoords( chooseVictim(victimsVector) );
	*target = chooseVictim(victimsVector);
	
	return true;
	
}


geometry_msgs::PoseStamped VictimTargetSelector::chooseVictim(std::vector<geometry_msgs::PoseStamped>& victimsVector ){
	
	int selectedTargetIndex = victimsVector.size()-1;

	std_msgs::Int16 indexMsg;
	indexMsg.data = selectedTargetIndex;
	_selectedTargetIndexPublisher.publish( indexMsg );
	
	return victimsVector[selectedTargetIndex];
	
}




