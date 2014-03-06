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
