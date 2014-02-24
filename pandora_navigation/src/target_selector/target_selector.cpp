#include "target_selector/target_selector.h"

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
