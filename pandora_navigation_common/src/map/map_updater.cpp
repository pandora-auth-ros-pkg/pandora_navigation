#include "pandora_navigation_common/map/map_updater.h"


MapUpdater::MapUpdater() : _mapAttributes(MAP_HEIGHT,MAP_WIDTH) {
	slamInitialized=false;

	ROS_DEBUG("Created MapUpdater instance");
		
}

void MapUpdater::startSubscriber(){
	
	std::string slamMapTopic;
	std::string robotPoseTopic;
		
	_robotPoseTimer = _nh.createTimer(ros::Duration(1),&MapUpdater::updateRobotPoseCallback,this);
	_robotPoseTimer.start();
	
	if (_nh.hasParam("slamMapTopic")) {
		_nh.getParam("slamMapTopic", slamMapTopic);
		ROS_DEBUG("[MapUpdater]: Got parameter slamMapTopic : %s" , slamMapTopic.c_str());
	}
	else {
		ROS_ERROR("[MapUpdater] : Parameter slamMapTopic not found. Using Default");
		slamMapTopic = "/slam/map" ;
	}
	
	_mapSubscriber = _nh.subscribe(slamMapTopic,1,&MapUpdater::updateMapCallback,this);
}

MapUpdater::~MapUpdater(){
	
	ROS_DEBUG("Destroyed MapUpdater instance");
	
}

void MapUpdater::updateRobotPoseCallback(const ros::TimerEvent&){

	while(!slamInitialized) return;
	
	tf::StampedTransform tfTransform;
	try {
		_listener.waitForTransform("world", "base_link", ros::Time(0), ros::Duration(1));
		_listener.lookupTransform("world", "base_link", ros::Time(0), tfTransform);
	}
	catch (tf::TransformException ex) {
		ROS_ERROR("[MapUpdater] Error in tf %d: %s",__LINE__, ex.what());
		return;
	}
	tf::Vector3 origin = tfTransform.getOrigin(); 
	
	tfScalar roll,pitch,yaw;
	tfTransform.getBasis().getRPY(roll,pitch,yaw);
	
	_mapAttributes.robotPose.dx= origin[0]/OCGD;
	_mapAttributes.robotPose.dy= origin[1]/OCGD;
	_mapAttributes.robotPose.dz= origin[2]/OCGD;
	_mapAttributes.robotPose.theta= yaw;
	_mapAttributes.robotPose.roll= roll;
	_mapAttributes.robotPose.pitch= pitch;
	
}
	
void MapUpdater::updateMapCallback(const nav_msgs::OccupancyGridConstPtr& msg){
	slamInitialized=true;
	
	int width=msg->info.width;
	int height=msg->info.height;
	
	float xmin=(msg->info.origin.position.x)/OCGD+MAP_SIZE/2;
	float ymin=(msg->info.origin.position.y)/OCGD+MAP_SIZE/2;
	float xmax=xmin+width;
	float ymax=ymin+height;
	
	_mapAttributes.prevxMin=xmin;
	_mapAttributes.prevxMax=xmax;
	_mapAttributes.prevyMin=ymin;
	_mapAttributes.prevyMax=ymax;
	
	for(int i=0;i<width;i++){
		for(int j=0;j<height;j++){
			_mapAttributes.map[i+_mapAttributes.prevxMin][j+_mapAttributes.prevyMin] = 
					(int)(100-msg->data[j*width+i])/100.0*255.0;
		}
	}
}
