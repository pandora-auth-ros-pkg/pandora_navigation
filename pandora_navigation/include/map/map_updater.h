#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>

#include "map_attributes.h"

#include "tf/transform_listener.h"

#ifndef MAP_SIZE_H
#define MAP_SIZE_H
	#define MAP_HEIGHT				4096		//!< Map height
	#define MAP_WIDTH				4096		//!< Map width
	#define MAP_SIZE				4096 		//!< Maximum map size
	#define	START_X					MAP_HEIGHT/2		//!< Start robot X coordinate in map
	#define	START_Y 				MAP_HEIGHT/2		//!< Start robot Y coordinate in map
	#define OCGD					0.02
#endif

class MapUpdater {
	
	public:
	
	bool slamInitialized;
	
	ros::Timer _robotPoseTimer;
	tf::TransformListener _listener;
	
	MapUpdater();
	virtual ~MapUpdater();
	
	void startSubscriber();
	MapAttributes& getMapAttributes() { return _mapAttributes ; }
		
	private:
	
	void updateMapCallback(const nav_msgs::OccupancyGridConstPtr& msg);
	
	void updateRobotPoseCallback(const ros::TimerEvent&);
	
	MapAttributes _mapAttributes;
	ros::Subscriber	_mapSubscriber;
	ros::Subscriber	_robotPoseSubscriber;
	ros::NodeHandle _nh;
};
