#include "target_selector/victim_target_selector.h"



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




