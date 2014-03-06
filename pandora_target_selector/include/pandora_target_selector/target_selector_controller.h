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
#include <actionlib/server/simple_action_server.h>
#include <std_srvs/Empty.h>
#include <target_selector_communications/SelectTargetAction.h>
#include <target_selector_communications/SelectTargetResult.h>
#include "pandora_target_selector/exploration_target_selector.h"
#include "pandora_target_selector/victim_target_selector.h"
#include "pandora_target_selector/target_selector_controller_defines.h"

#include "geometry_msgs/PoseStamped.h"

#include "pandora_navigation_communications/NavigationGeotiffSrv.h"

#include <nav_msgs/OccupancyGrid.h>

class TargetSelectorController{


public:
	MapUpdater _mapUpdater;
	
	//~ ros::Subscriber robotPoseSubscriber;
	void patchCoverage(const ros::TimerEvent&);
	
	TargetSelectorController(void);
	MapAttributes& mapAttributes;
	Coverage coverage;
	
	void coverageTimerCallback(const ros::TimerEvent&);
	
	ros::Timer _coverageCallbackTimer;
	ros::Timer _coveragePatchingTimer;
	ros::Publisher _occupancyGridPublisher;
	//~ ros::Publisher takis;

private:
	
	TargetSelector* targetSelector;
	VictimTargetSelector victimTargetSelector;
	ExplorationTargetSelector explorationTargetSelector;
	ClosestUnexploredTargetSelector closestUnexploredTargetSelector;
	
	geometry_msgs::PoseStamped transformPixelCoords2PoseStamped(PixelCoords pixelCoords);
	
	bool sendMap(pandora_navigation_communications::navigationMapSrv::Request &req,pandora_navigation_communications::navigationMapSrv::Response &res);
	void makeDummyCoveragePatch(unsigned char **map,unsigned char **coverage,int x, int y,int d);
	bool flushCoverage(std_srvs::Empty::Request& rq,std_srvs::Empty::Response &rs);
	
	bool checkPreempt(void);

	
	
protected:
	ros::NodeHandle nh_;
	
	ros::ServiceServer _mapService;
	ros::ServiceServer geotiffSrv;
	
	ros::ServiceServer _flushService;
	
	// NodeHandle instance must be created before this line. Otherwise strange error may occur.
	actionlib::SimpleActionServer<target_selector_communications::SelectTargetAction> selectTargetActionServer_; 
	std::string action_name_;
	
	void selectTargetActionCallBack(const target_selector_communications::SelectTargetGoalConstPtr &goal);
	bool geotiffSericeCb(pandora_navigation_communications::NavigationGeotiffSrv::Request &req,pandora_navigation_communications::NavigationGeotiffSrv::Response &res);
	

};

