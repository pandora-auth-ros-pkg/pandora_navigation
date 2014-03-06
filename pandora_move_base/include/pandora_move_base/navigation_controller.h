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

#ifndef NAVIGATION_CONTROLLER_H
#define NAVIGATION_CONTROLLER_H

#include "pandora_move_base/navigation_controller_defines.h"

class NavigationController{
	
public:
	LaserInfo lsr;
	NavigationController(void);
	
private:
	MapUpdater _mapUpdater;
	PandoraNavigator navigator;
	PandoraPathPlanner pandoraPathPlanner;
	MapAttributes& mapAttributes;
	
	ros::Subscriber _sonarSubscriber;
	ros::Subscriber _headSonarSubscriber;
	ros::Subscriber _laserScansSubscriber;
	ros::Subscriber _compassSubscriber;
	// Path
	//~ std::vector<PixelCoords> pixelPlan;				//!< Vector with path to target
	PixelCoords pixelGoal;							//!<Target
	bool performInitial;
	geometry_msgs::PoseStamped poseStampedGoal;
	std::vector<geometry_msgs::PoseStamped> poseStampedPlan;
	std::vector<geometry_msgs::PoseStamped>* poseStampedPlanPointer;
	
	LaserScan scan;				//!< The current laser scan
	SonarScan sonars;    		//!< The current sonar scan
	IrScan irs;   				//!< The current IR scan
	float compassPitch;		//!< Holds the compass pitch
	
	
	
	float expectedDx;
	float expectedDy;
	float expectedDtheta;
	int stuckCounter;
	
	bool stuckReset;
	Transformation expectedRobotPose;
	
	
	bool calcPath( const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& poseStampedPlan );
	void sendSpeedsToMotors(geometry_msgs::Twist twistVelocities);
	
	int moveToFinalTarget(std::vector<geometry_msgs::PoseStamped>& poseStampedPlan);
	int moveToTargetWithSubGoals(std::vector<geometry_msgs::PoseStamped>& poseStampedPlan);
	int moveToTargetRecalculate(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& poseStampedPlan);
	int moveToTargetRecalculate_stuckAvoidance(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& poseStampedPlan);
	
	bool performInitialTurn(void);
	int parkRobot(float theta);
	float calcParkingAngle(float parkOrientation);
	
	void stopMotors(void);
	void rotateRobotToUnstuck(void);
	void moveRobotToUnstuck(void);
	
	
	//~ void updateRobotPose(const slam_communications::robotPoseMsg& msg);
	//~ void updateIRs(const controllers_and_sensors_communications::irMsg& msg);
	void updateSonars(const sensor_msgs::Range& msg);
	void updateLaserScans(const sensor_msgs::LaserScanConstPtr& msg);
	void serveCompassMessage(const controllers_and_sensors_communications::compassMsg& msg);
	
	PixelCoords transformPoseStamped2PixelCoords(geometry_msgs::PoseStamped point);
	geometry_msgs::PoseStamped transformPixelCoords2PoseStamped(PixelCoords point);
	void transformTwist2Velocities( geometry_msgs::Twist twist, float* linear, float* rotational);
	
	bool sendMap(pandora_navigation_communications::navigationMapSrv::Request &req,pandora_navigation_communications::navigationMapSrv::Response &res);
	
	bool checkPreempt(void);
	
	void publishPath(const ros::TimerEvent&); 
	
protected:
	ros::NodeHandle nh_;
	
	
	ros::Publisher _velocityPublisher;
	ros::Publisher _pathPublisher;
	ros::ServiceServer _mapService;
	
	ros::Timer _pathStreamingTimer;
	
	// NodeHandle instance must be created before this line. Otherwise strange error may occur.
	actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> moveBaseActionServer_; 
	actionlib::SimpleActionServer<pandora_navigation_communications::InitialTurnAction> initialTurnActionServer_; 
	actionlib::SimpleActionServer<pandora_navigation_communications::RobotStuckAction> robotStuckActionServer_; 
	std::string action_name_;
	
	void moveBaseActionCallBack(const move_base_msgs::MoveBaseGoalConstPtr &goal);
	void initialTurnActionCallBack(const pandora_navigation_communications::InitialTurnGoalConstPtr &goal);
	void robotStuckActionCallBack(const pandora_navigation_communications::RobotStuckGoalConstPtr &goal);
	
};

#endif

