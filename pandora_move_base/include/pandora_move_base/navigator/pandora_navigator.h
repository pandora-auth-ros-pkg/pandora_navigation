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

#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include "navigator_defines.h"

//~ #include "geometry_msgs/PoseStamped.h"
//~ #include "base_local_planner.h"
//~ #include "misc/pixelcoords.h"
//~ #include "map/transformation.h"
//~ #include "misc/navigator/ir_scans.h"
//~ #include "misc/navigator/laser_scans.h"
//~ #include "misc/navigator/sonar_scan.h"
//~ #include "misc/navigator/point.h"

using namespace nav_core;

struct LaserInfo{
	int sLASER_RAYS;
	float sD_LASER_MAX;
	float sD_LASER_ANG;
	float sD_LASER_ANG_RAD;
	float sD_LASER_ANGLE_BEGIN;
	float sD_LASER_ANGLE_END;
	float sD_LASER_COEFF_FIX;
};

class PandoraNavigator: public BaseLocalPlanner{
	
public:
	
	const LaserInfo& lsr;
	
	bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
	//bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
	bool isGoalReached();
	//void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros){}
	bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);
	
	
	
	void moveToTarget( std::vector<PixelCoords> goal );
	//~ void performInitialTurn(void);
	void performInitialTurn(geometry_msgs::Twist& cmd_vel);
	void parkRobot(  geometry_msgs::Twist& cmd_vel, float angleToTurn  );
	float rotateToUnstuck(void);
	
	PixelCoords transformPoseStamped2PixelCoords(geometry_msgs::PoseStamped point);
	geometry_msgs::PoseStamped transformPixelCoords2PoseStamped(PixelCoords point);
	geometry_msgs::Twist transformVelocities2Twist(float linear, float rotational);
	
	
	std::vector<PixelCoords> pixelPlan;
	//unsigned int currGoal;	//!< The current goal's index
	bool multipleSubgoals;
	//~ unsigned int pathSize;


	
	//~ bool isSubgoalReached;			//!< True if the current subgoal is reached
	//~ bool isTargetLocked;			//!< True if the robot has a target
	//~ bool pathWithSemiAutonomous;	//!< True if the robot must follow a path in semi-autonomous mode
	//~ bool pathSrvNew;				//!< True if pathSrv needs to be active
	//~ bool pathSrvPrev;				//!< True if pathSrv was active
	//~ bool motorSpeedNew;				//!< True if motor speed srv needs to be active
	//~ bool motorSpeedPrev;			//!< True if motor speed srv was active
	//~ bool pathFinishedNew;			//!< True if path finished srv needs to be active
	//~ bool pathFinishedPrev;			//!< True if path finished srv was active
	//~ bool robotPoseNew;				//!< True if robotPose subscriber needs to be active
	//~ bool robotPosePrev;				//!< True if robotPose subscriber was active
	//~ bool laserScanNew;				//!< True if laser scan subscriber needs to be active
	//~ bool laserScanPrev;				//!< True if laser scan subscriber needs to be active
	//~ bool sonarsNew;					//!< True if sonars subscriber needs to be active
	//~ bool sonarsPrev;				//!< True if sonars subscriber was active
	//~ bool irsNew;					//!< True if IR's subscriber needs to be active
	//~ bool irsPrev;					//!< True if IR's subscriber was active
	//~ bool visionLineColorNew;		//!< True if visionLineColor subscriber needs to be active
	//~ bool visionLineColorPrev;		//!< True if visionLineColor subscriber was active
	bool isRobotStarted;     		//!<	True if the robot has just started operating
	//~ bool cameraAlert; 				//!<	True if the path was created from camera alert
	//~ bool isLinearizing;				//!<	True if the robot is in the linearizing state
	//~ bool hold;

	//~ char **argv;			//!< The input variables of the program
	
	//~ int argc;				//!< The number of program arguments	
	//~ int timer;				//!< Timer that holds the elapsed time since the path was given
	int firstTime;			//!< Holds if the path is a first oneLaserInfo
	//~ int timeoutSec;			//!< Variable for Timeout for subgoals
	//~ int finishedState;
	
	unsigned int currGoal;	//!< The current goal's index
	
	float *cosTheta;		//!< Look Up Table for cosine
	float *sinTheta;		//!< Look Up Table for sine
	float rotationalSpeed;	//!< Rotational speed of the vehicle
	float linearSpeed;		//!< Linear speed of the vehicle
	float angleToGoal;		//!< Finds the relative angle from robot to goal
	
	//~ float linearizeAngle;	//!< Calculated angle to linearize close to victim
	//~ float colorLineTheta;	//!< Angle of the colored line
	//~ float colorIntensity;	//!< Intensity of the line - Length
	//~ 
	//~ double lastSeenColor;	//!< Time for last seen color
	//~ 
	Transformation& robotPose;	//!< Holds the robot pose
	float& compassPitch;		//!< Holds the compass pitch	
	LaserScan& scan;				//!< The current laser scan
	SonarScan& sonars;    		//!< The current sonar scan
	IrScan& irs;  
	
	tf::TransformListener _listener;
	//~ std::vector<PixelCoords> goal;	//!< Vector with subgoals
	
	
	
	/**
	@brief Default costructor
	@param argc [int] The number of input arguments
	@param argv [char **] The input arguments
	@return void
	**/
	//~ PandoraNavigator (int argc, char **argv);
	PandoraNavigator (Transformation& robotPose, LaserScan& scan, SonarScan& sonars, IrScan& irs, float& compassPitch,const LaserInfo &lsr_);
	
	/**
	@brief Destructor
	@param void
	@return void
	**/
	~PandoraNavigator (){}
	
	
	void getPitchAndYaw(float* pitchPtr,float* yawPtr);
	
	void stopMotors(void);
	
	//~ float performRotationalCorrection(void);
	float performRotationalCorrection(void);
	
	float performLinearCorrection(void);
	
	float performCircleCorrection(bool stuck=false);
	
	//~ void updateRobotPose(const slam_communications::robotPoseMsg& msg);
	//~ 
	//~ void updateIRs(const controllers_and_sensors_communications::irMsg& msg);
	//~ 
	//~ void updateSonars(const controllers_and_sensors_communications::sonarMsg& msg);
	//~ 
	//~ void updateLaserScans(const slam_communications::laserScanMsg& msg);
	//~ 
	//~ void serveCompassMessage(const controllers_and_sensors_communications::compassMsg& msg);
	
	
	//~ actionlib::SimpleActionClient<main_motor_control_communications::setVehicleSpeedAction> _setVehicleSpeedClient;
	//~ main_motor_control_communications::setVehicleSpeedGoal _goal;	
	float calculateTurnCoefficient(PixelCoords rPose, PixelCoords sgPose,float rYaw);
};


#endif

