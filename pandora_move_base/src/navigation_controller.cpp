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

#include "pandora_move_base/navigation_controller.h"


NavigationController::NavigationController(void):
		mapAttributes(_mapUpdater.getMapAttributes()),
		pandoraPathPlanner(&(_mapUpdater.getMapAttributes())),
		navigator((_mapUpdater.getMapAttributes()).robotPose, scan, sonars, irs, compassPitch,lsr),
		moveBaseActionServer_(nh_, "move_base", boost::bind(&NavigationController::moveBaseActionCallBack,
			this, _1), false),
		initialTurnActionServer_(nh_, "initial_turn", boost::bind(&NavigationController::initialTurnActionCallBack,
			this, _1), false),
		robotStuckActionServer_(nh_, "robot_stuck", boost::bind(&NavigationController::robotStuckActionCallBack,
			this, _1), false)
{
	_velocityPublisher = nh_.advertise<geometry_msgs::Twist>("/cmd_vel",1000);
	
	 _mapService= nh_.advertiseService("NavigationMapSrv/navigationController",&NavigationController::sendMap,this);
	
	_pathPublisher = nh_.advertise<nav_msgs::Path>("planner_path",1000);
	
		//~ std::cout<<"koukou!\n cha!\n";
	//~ action_name_(name)
	moveBaseActionServer_.start();
	initialTurnActionServer_.start();
	robotStuckActionServer_.start();
	
	//~ mapAttributes(_mapUpdater.getMapAttributes() );
	_mapUpdater.startSubscriber();
	while(!_mapUpdater.slamInitialized && ros::ok()){
		ros::Duration(1).sleep();
		ros::spinOnce();
	}

	std::string compassTopic;
	std::string sonarTopic;
	std::string headSonarTopic;
	std::string laserScansTopic;
	//laser scans
	if (nh_.hasParam("laserScansTopic")) {
		nh_.getParam("laserScansTopic", laserScansTopic);
		ROS_DEBUG("[NavigationController]: Got parameter laserScansTopic : %s" , laserScansTopic.c_str());
	}
	else {
		ROS_WARN("[NavigationController] : Parameter laserScansTopic not found. Using Default");
		laserScansTopic = "/slam/scan" ;
	}
	//sonar
	if (nh_.hasParam("sonarTopic")) {
		nh_.getParam("sonarTopic", sonarTopic);
		ROS_DEBUG("[NavigationController]: Got parameter sonarTopic : %s" , sonarTopic.c_str());
	}
	else {
		ROS_WARN("[NavigationController] : Parameter sonarTopic not found. Using Default");
		sonarTopic = "/sensors/sonar" ;
	}
	//head Sonar
	if (nh_.hasParam("headSonarTopic")) {
		nh_.getParam("headSonarTopic", headSonarTopic);
		ROS_DEBUG("[NavigationController]: Got parameter headSonarTopic : %s" , headSonarTopic.c_str());
	}
	else {
		ROS_WARN("[NavigationController] : Parameter headSonarTopic not found. Using Default");
		headSonarTopic = "/sensors/headSonar" ;
	}
	//compass
	if (nh_.hasParam("compassTopic")) {
		nh_.getParam("compassTopic", compassTopic);
		ROS_DEBUG("[NavigationController]: Got parameter compassTopic : %s" , compassTopic.c_str());
	}
	else {
		ROS_WARN("[NavigationController] : Parameter compassTopic not found. Using Default");
		compassTopic = "/sensors/compass" ;
	}
	
	_laserScansSubscriber = nh_.subscribe( laserScansTopic , 1 , &NavigationController::updateLaserScans , this );
	_sonarSubscriber = nh_.subscribe(sonarTopic, 1, &NavigationController::updateSonars, this);
	_compassSubscriber = nh_.subscribe(compassTopic, 1, &NavigationController::serveCompassMessage, this);


	
	//TODO:implementation
	//~ partitionGraphPathPlanner = new PartitionGraphPathPlanner;
	//pandoraPathPlanner = new PandoraPathPlanner;
	
	//~ pixelPlan.clear();
	pixelGoal.setXCoord(-1);
	pixelGoal.setYCoord(-1);
	performInitial = true;
	stuckReset = true;
	
	
	_pathStreamingTimer = nh_.createTimer(ros::Duration(1),&NavigationController::publishPath,this);
	_pathStreamingTimer.start();
	
	//~ poseStampedPlan.clear();
	//~ #ifdef FLOW
		//~ ROS_INFO("[NAVIGATION CONTROLLER %d]: Leaving Constructor",__LINE__);
	//~ #endif
	//~ linearSpeed = rotationalSpeed = 0;
	
	//~ TODO: initialize the rest of the params
	
	sonars.arm=10.0;
	sonars.right=10.0;
	sonars.left=10.0;
	sonars.backLeft=10.0;
	sonars.backRight=10.0;
	sonars.front=10.0;
}
//~ void NavigationController::initialTurnActionCallBack(void){
void NavigationController::initialTurnActionCallBack(const pandora_navigation_communications::InitialTurnGoalConstPtr &goal){
	bool initialTurnStatus;
	if (performInitial){
		//~ ROS_INFO_NAMED("navigation_controller", "[navigation_controller %d]: Performing initial turn",__LINE__);
		initialTurnStatus = performInitialTurn();
		performInitial=false;
		if (initialTurnStatus==true){
			//~ ROS_INFO_NAMED("navigation_controller", "[navigation_controller %d]: Initial turn successfull",__LINE__);
			initialTurnActionServer_.setSucceeded();
		}
		else{
			//~ ROS_ERROR_NAMED("navigation_controller", "[navigation_controller %d]: Initial turn preempted",__LINE__);
			initialTurnActionServer_.setPreempted();
		}
	}
	else{
		//~ ROS_ERROR_NAMED("navigation_controller", "[navigation_controller %d]: Request for initial turn rejected",__LINE__);
		initialTurnActionServer_.setAborted();
	}
}

void NavigationController::moveBaseActionCallBack(const move_base_msgs::MoveBaseGoalConstPtr &goal){
	std::vector<PixelCoords> pixelPlan;
	pixelPlan.clear();
	//~ std::vector<geometry_msgs::PoseStamped> poseStampedPlan;
	poseStampedPlan.clear();
	geometry_msgs::PoseStamped start;
	geometry_msgs::PoseStamped target;
	
	int goalReached;
	
	//---transform meters to pixels according to occupancy grid map resolution
	target.pose.position.x = (int) (goal->target_pose.pose.position.x / 0.02) + START_X;
	target.pose.position.y = (int) (goal->target_pose.pose.position.y / 0.02) + START_Y;
	
	ROS_INFO_NAMED("navigation_controller", "[navigation_controller %d]: Calculating Path",__LINE__);
	
	if ( calcPath( start, target, poseStampedPlan) ){
		#ifdef FLOW
		for (int i=0; i<poseStampedPlan.size();i++){
			ROS_INFO_NAMED("navigation_controller_path", "[navigation_controller %d]: Path point[%d] x=%f y=%f",i,poseStampedPlan[i].pose.position.x,poseStampedPlan[i].pose.position.y);
		}
		#endif
		
		
		//path found. move to target
		#ifdef MOVE_WITH_SUBGOALS
			ROS_INFO_NAMED("navigation_controller", "[navigation_controller %d]: Moving with subgoals",__LINE__);
			goalReached = moveToTargetWithSubGoals(poseStampedPlan);
		#elif defined MOVE_WITH_PATH_RECALCULATION
			ROS_INFO_NAMED("navigation_controller", "[navigation_controller %d]: Moving with path recalculation",__LINE__);
			goalReached = moveToTargetRecalculate(target, poseStampedPlan);
		#elif defined MOVE_WITH_PATH_RECALCULATION_STUCK_AVOIDANCE
			ROS_INFO_NAMED("navigation_controller", "[navigation_controller %d]: Moving with path recalculation and robot stuck detection",__LINE__);
			goalReached = moveToTargetRecalculate_stuckAvoidance(target, poseStampedPlan);
		#elif defined MOVE_TO_FINAL_TARGET
			ROS_INFO_NAMED("navigation_controller", "[navigation_controller %d]: Moving to final target",__LINE__);
			goalReached = moveToFinalTarget(poseStampedPlan);
		#else
			//~ ROS_ERROR_NAMED("navigation_controller", "[navigation_controller %d]: NO NAVIGATOR MODE SELECTED",__LINE__);
			moveBaseActionServer_.setAborted();
			return;
			//error naoum
		#endif
		
		if ( goalReached==1 ) {
			//final goal reached
			ROS_INFO_NAMED("navigation_controller", "[navigation_controller %d]: Final Goal Reached",__LINE__);
			
			float quaTx = goal->target_pose.pose.orientation.x;
			float quaTy = goal->target_pose.pose.orientation.y;
			float quaTz = goal->target_pose.pose.orientation.z;
			float quaTw = goal->target_pose.pose.orientation.w;
			
			int parkRobotStatus;
			
			if( quaTx!=0 || quaTy!=0 || quaTz!=0 || quaTw!=0){
				//park robot
				ROS_INFO_NAMED("navigation_controller", "[navigation_controller %d]: Parking robot",__LINE__);
				tfScalar roll,pitch,yaw;
				tf::Matrix3x3(tf::Quaternion(quaTx,quaTy,quaTz,quaTw)).getRPY(roll,pitch,yaw);
				
				float theta = yaw;
				parkRobotStatus = parkRobot(theta);
				
				if (parkRobotStatus==1){ //robot is parked. everything ok
					//~ ROS_ERROR_NAMED("navigation_controller", "[navigation_controller %d]: Parking finished", __LINE__);
					moveBaseActionServer_.setSucceeded();
				}
				else if (parkRobotStatus==3){ //preempted while parking
					//~ ROS_ERROR_NAMED("navigation_controller", "[navigation_controller %d]: Parking was cancelled",__LINE__);
					moveBaseActionServer_.setPreempted();
				}
				
			}
			else{
				//~ ROS_ERROR_NAMED("navigation_controller", "[navigation_controller %d]: Navigation Finished", __LINE__);
				moveBaseActionServer_.setSucceeded();
			}
			
		}
		else if (goalReached==2 ){
			//goal time-out
			//~ ROS_ERROR_NAMED("navigation_controller", "[navigation_controller %d]: Goal time-out",__LINE__);
			moveBaseActionServer_.setAborted();
		}
		else if (goalReached==3 ){
			//preempted while navigating
			//~ ROS_ERROR_NAMED("navigation_controller", "[navigation_controller %d]: Navigation was cancelled",__LINE__);
			moveBaseActionServer_.setPreempted();
		}
	}
	else{
		// no path found. send error message
		//~ ROS_ERROR_NAMED("navigation_controller", "[navigation_controller %d]: Path NOT FOUND!", __LINE__);
		moveBaseActionServer_.setAborted();
	}
	
}


void NavigationController::robotStuckActionCallBack(const pandora_navigation_communications::RobotStuckGoalConstPtr &goal){
	
	//~ ROS_ERROR_NAMED("robot_stuck","[navigation_controller %d]: Robot stuck. performing turn with rotational correction",__LINE__);
	#ifdef SIMPLE_UNSTUCK_ROTATION
		rotateRobotToUnstuck();
	#elif defined UNSTUCK_ROTATION_WITH_CORRECTIONS
		moveRobotToUnstuck();
	#endif
	
	robotStuckActionServer_.setSucceeded();
}


void NavigationController::rotateRobotToUnstuck(void){
	
	//~ float rotational,linear=0;
	geometry_msgs::Twist twistVelocities;
	
	twistVelocities.linear.x =0;
	twistVelocities.angular.z = navigator.rotateToUnstuck();
	//~ ROS_ERROR_NAMED("robot_stuck","[navigation_controller %d]: Rotational Velocity is = %f",__LINE__,twistVelocities.angular.z );
	sendSpeedsToMotors(twistVelocities);
	
	usleep(2500000);
	
	stopMotors();
}


void NavigationController::moveRobotToUnstuck(void){
	
	//~ float rotational,linear=0;
	geometry_msgs::Twist twistVelocities;
	bool tempStuck=true;
	
	for (int i=0; i<25; i++){
		twistVelocities.linear.x =navigator.performCircleCorrection(tempStuck);
		twistVelocities.angular.z = navigator.rotateToUnstuck();
		//~ ROS_ERROR_NAMED("robot_stuck","[navigation_controller %d]: Rotational Velocity is = %f  Linear Velocity is = %f",__LINE__,twistVelocities.angular.z, twistVelocities.linear.x );
		sendSpeedsToMotors(twistVelocities);
		
		//~ usleep(2500000);
		usleep(100000);
	}	
	
	stopMotors();
}


int NavigationController::parkRobot(float parkOrientation){
	
	geometry_msgs::Twist twistVelocities;
	ROS_INFO_NAMED("navigation_controller", "[navigation_controller %d]: Calculating parking angle", __LINE__);
	float finalAngle = calcParkingAngle(parkOrientation);
	
	
	ROS_INFO_NAMED("navigation_controller", "[navigation_controller %d]: Found parking angle: %f", __LINE__,finalAngle);
	int secs=fabs(finalAngle)/0.05;
	//~ int secs=fabs(parkOrientation)/0.10;
	
	for(unsigned int kk=0;kk<secs*2;kk++){
	//~ for(unsigned int kk=0;kk<secs*10;kk++){
	//~ while (1){
		
		navigator.parkRobot(twistVelocities, finalAngle);
		
		sendSpeedsToMotors(twistVelocities);
		
		if ( fabs(finalAngle-mapAttributes.robotPose.theta) < D_PI/36 ){
			ROS_WARN_NAMED("navigation_controller", "[navigation_controller %d]: Parking Angle below threshold.", __LINE__);
			break;
		}
		
		if ( checkPreempt() ){
			//~ ROS_ERROR_NAMED("navigation_controller", "[navigation_controller %d]: Parking Preempted. STOPPING MOTORS", __LINE__);
			stopMotors();
			//~ return false;
			return 3;
		}
		usleep(500000);
		//~ usleep(100000);
		//~ ROS_INFO("[NAVIGATOR @ linearize] Time : %f",kk/2.0);
	}
	
	stopMotors();
	return 1;
}


float NavigationController::calcParkingAngle(float parkOrientation){
	
	float temp=parkOrientation-mapAttributes.robotPose.theta;
	float finalAngle=0;
	//~ ROS_INFO("[PLANNER]Angle of RobotToVictim is : %f",theta1);
	//~ ROS_INFO("[PLANNER]Angle of Robot is : %f ",theta2);
	//~ ROS_INFO("[PLANNER]Angle Between : %f ",temp);
	ROS_INFO_NAMED("navigation_controller", "[navigation_controller %d]: Angle Between : %f ", __LINE__,temp);
	if(fabs(temp)>D_PI && temp>0)
		finalAngle=-(2*D_PI-fabs(temp));
	else if(fabs(temp)>D_PI && temp<0)
		finalAngle=(2*D_PI-fabs(temp));
	else if(fabs(temp)<D_PI)
		finalAngle=temp;
	
	//change orientation
	if ( fabs(finalAngle) > (D_PI/2) ){
		
		finalAngle += D_PI;
		
		if(fabs(finalAngle)>D_PI && finalAngle>0)
			finalAngle=-(2*D_PI-fabs(finalAngle));
		else if(fabs(finalAngle)>D_PI && finalAngle<0)
			finalAngle=(2*D_PI-fabs(finalAngle));
		//~ else if(fabs(finalAngle)<D_PI)
			//~ finalAngle=finalAngle;
	
	}
	//~ ROS_INFO("[PLANNER]Angle Between : %f ",temp);
	return finalAngle;
	
}


bool NavigationController::performInitialTurn(void){
	geometry_msgs::Twist twistVelocities;
	int firstTime=0;
	
	
	while(firstTime<20 && performInitial){
		
		navigator.performInitialTurn(twistVelocities);
		
		sendSpeedsToMotors(twistVelocities);
		if ( checkPreempt() ){
			//~ ROS_ERROR_NAMED("navigation_controller", "[navigation_controller %d]: Initial turn Preempted. STOPPING MOTORS", __LINE__);
			stopMotors();
			//~ return false;.
			return false;
		}
		
		usleep(400000);
		firstTime++;
	}
	stopMotors();
	return true;
}



//~ std::vector<PixelCoords> NavigationController::calcPath(PixelCoords goal){
//~ void NavigationController::calcPath(void){
bool NavigationController::calcPath(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& poseStampedPlan){
	
	if ( !pandoraPathPlanner.makePlan( start, goal, poseStampedPlan) ){
		//~ pixelPlan.clear();
		//~ ROS_ERROR_NAMED("navigation_controller", "[navigation_controller %d]: calc path. path not found",__LINE__);
		//no path found
		poseStampedPlan.clear();
		return false;
	}
	//~ else{
		//~ pixelPlan.clear();
		//~ for (unsigned int i=0; i, poseStampedPlan.size(); i++)
			//~ pixelPlan.push_back( transformPoseStamped2PixelCoords(poseStampedPlan[i]) );
	//~ }
	//~ ROS_ERROR_NAMED("navigation_controller", "[navigation_controller %d]: calc path. path found",__LINE__);
	//path found
	return true;
}


//~ void NavigationController::getSpeeds(std::vector<PixelCoords>& pixelPlan){
int NavigationController::moveToFinalTarget(std::vector<geometry_msgs::PoseStamped>& poseStampedPlan){
	int timeOutCounter=0, finalGoalTimeOut;
	
	finalGoalTimeOut = TIME_OUT_THRESHOLD * poseStampedPlan.size();
	
	ROS_INFO_NAMED("navigation_controller", "[navigation_controller %d]: MOVE_TO_FINAL_TARGET",__LINE__);
	
	geometry_msgs::Twist twistVelocities;
	
	
	//~ navigator.setPlan( &poseStampedPlan[0] );
	ROS_INFO_NAMED("navigation_controller", "[navigation_controller %d]: Setting plan to navigator",__LINE__);
	navigator.setPlan( poseStampedPlan );
	
	ROS_INFO_NAMED("navigation_controller", "[navigation_controller %d]: Plan to navigator is set",__LINE__);
	
	while ( !navigator.isGoalReached() ){
		
		
		//~ ROS_INFO_NAMED("navigation_controller", "[navigation_controller %d]: Request velocities from navigator",__LINE__);
		if ( timeOutCounter == finalGoalTimeOut ){
			//~ ROS_ERROR_NAMED("navigation_controller", "[navigation_controller %d]: Time out", __LINE__);
			stopMotors();
			//~ return false;
			return 2;
		}
		timeOutCounter++;
		
		if ( checkPreempt() ){
			//~ ROS_ERROR_NAMED("navigation_controller", "[navigation_controller %d]: Navigation Preempted. STOPPING MOTORS", __LINE__);
			stopMotors();
			//~ return false;
			return 3;
		}
		
		if ( !navigator.computeVelocityCommands(twistVelocities) ){
			//~ ROS_ERROR_NAMED("navigation_controller", "[navigation_controller %d]: No velocities returned",__LINE__);
			//something went wrong with the navigator
			twistVelocities.linear.x =0;
			twistVelocities.angular.z =0;
			//~ stopMotors();
			//~ return false;
		}
		
		sendSpeedsToMotors(twistVelocities);
		usleep(VELOCITIES_REQUEST_INTERVAL);
	}
	
	stopMotors();
	return 1;
}

int NavigationController::moveToTargetWithSubGoals(std::vector<geometry_msgs::PoseStamped>& poseStampedPlan){
	
	int timeOutCounter = 0;
	geometry_msgs::Twist twistVelocities;
	unsigned int currGoal=0;
	std::vector<geometry_msgs::PoseStamped> poseStampedSubGoal;
	poseStampedSubGoal.clear();
	
	poseStampedSubGoal.push_back(poseStampedPlan[currGoal]);
	
	
	navigator.setPlan( poseStampedSubGoal );
	
	while (1){
		
		if ( navigator.isGoalReached() ){
			timeOutCounter = 0;
			
			ROS_INFO_NAMED("navigation_controller", "[navigation_controller %d]: Subgoal Reached", __LINE__);
			
			currGoal++;
			if(currGoal==poseStampedPlan.size()){	//final goal reached
				ROS_INFO_NAMED("navigation_controller", "[navigation_controller %d]: Final Goal reached", __LINE__);
				break;
			}
			else{									//move to next subgoal
				ROS_INFO_NAMED("navigation_controller", "[navigation_controller %d]: Move to next subgoal", __LINE__);
				//TODO: reinitialize timer
				poseStampedSubGoal.clear();
				poseStampedSubGoal.push_back(poseStampedPlan[currGoal]);
				#ifdef FLOW
					ROS_INFO("[NAVIGATION CONTROLLER]: Path point x=%f y=%f",poseStampedPlan[currGoal].pose.position.x,poseStampedPlan[currGoal].pose.position.y);
				#endif
				navigator.setPlan( poseStampedSubGoal );
			}
		}
		else if ( timeOutCounter == TIME_OUT_THRESHOLD ){
			//~ ROS_ERROR_NAMED("navigation_controller", "[navigation_controller %d]: Time out", __LINE__);
			stopMotors();
			//~ return false;
			return 2;
		}
		timeOutCounter++;
		
		
		if ( checkPreempt() ){
			//~ ROS_ERROR_NAMED("navigation_controller", "[navigation_controller %d]: Navigation Preempted. STOPPING MOTORS", __LINE__);
			stopMotors();
			//~ return false;
			return 3;
		}
		
		if ( !navigator.computeVelocityCommands(twistVelocities) ){
			//something went wrong with the navigator
			//~ ROS_ERROR_NAMED("navigation_controller", "[navigation_controller %d]: Error requesting velocities. STOPPING MOTORS", __LINE__);
			twistVelocities.linear.x =0;
			twistVelocities.angular.z =0;
			//~ stopMotors();
			//~ return false;
		}
		
		sendSpeedsToMotors(twistVelocities);
		usleep(VELOCITIES_REQUEST_INTERVAL);
	}
	
	stopMotors();
	return 1;
}

int NavigationController::moveToTargetRecalculate( const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& poseStampedPlan){
	int timeOutCounter = 0;
	
	geometry_msgs::Twist twistVelocities;
	unsigned int currGoal=0;
	geometry_msgs::PoseStamped start;
	std::vector<geometry_msgs::PoseStamped> poseStampedSubGoal;
	poseStampedSubGoal.clear();
	PixelCoords pixelGoal=transformPoseStamped2PixelCoords(poseStampedPlan[currGoal]);
	
	ROS_INFO_NAMED("navigation_controller", "[navigation_controller %d]: Selecting first subgoal", __LINE__);
	
	while ( (poseStampedPlan[currGoal].pose.position.x == mapAttributes.robotPose.dx+START_X && poseStampedPlan[currGoal].pose.position.y == mapAttributes.robotPose.dy+START_Y  ) ||  pixelGoal.computeDistanceFrom(PixelCoords(mapAttributes.robotPose.dx+START_X, mapAttributes.robotPose.dy+START_Y)) <= DISTANCE_TO_APPROACH ){
		currGoal++;
		
		//TODO: maybe this needs to go
		if ( currGoal > poseStampedPlan.size()-1 ){
			//~ ROS_ERROR_NAMED("navigation_controller", "[navigation_controller %d]: TOO CLOSE TO TARGET. considered visited", __LINE__);
			return 1;
		}
		
		pixelGoal=transformPoseStamped2PixelCoords(poseStampedPlan[currGoal]);
	}
	poseStampedSubGoal.push_back(poseStampedPlan[currGoal]);
	//~ start = poseStampedPlan[currGoal];
	#ifdef FLOW
		//~ ROS_INFO("[NAVIGATION CONTROLLER %d]: robotPose  x=%d y=%d",__LINE__,mapAttributes.robotPose.dx+START_X,mapAttributes.robotPose.dy+START_Y);
		//~ ROS_INFO("[NAVIGATION CONTROLLER %d]: goal position x=%f y=%f",__LINE__,poseStampedPlan[currGoal].pose.position.x,poseStampedPlan[currGoal].pose.position.y);
		//~ ROS_INFO("[NAVIGATION CONTROLLER %d]: curr goal=%d DISTANCE_TO_APPROACH=%d",__LINE__,currGoal,DISTANCE_TO_APPROACH);
	#endif
	navigator.setPlan( poseStampedSubGoal );
	
	while (1){
		if ( navigator.isGoalReached() ){
			//reset timeout counter
			timeOutCounter = 0;
			
			ROS_INFO_NAMED("navigation_controller", "[navigation_controller %d]: Subgoal Reached", __LINE__);
			
			 
			if ( poseStampedPlan[currGoal].pose.position.x==goal.pose.position.x && poseStampedPlan[currGoal].pose.position.y==goal.pose.position.y ){//final goal reached. Success
				
				ROS_INFO_NAMED("navigation_controller", "[navigation_controller %d]: Final Goal reached", __LINE__);
				break; 
			}
			
			//recalculate path
			ROS_INFO_NAMED("navigation_controller", "[navigation_controller %d]: Recalculating path", __LINE__);
			if ( !pandoraPathPlanner.makePlan( start, goal, poseStampedPlan) ){//could NOT recalculate path!!!!
			
				//~ ROS_ERROR_NAMED("navigation_controller", "[navigation_controller %d]: Could NOT recalculate path", __LINE__);
				stopMotors();
				//~ return false;
				return 2;
			}
			else{ //~ path recalculated. new subGoal found
				//TODO: reinitialize timer
				ROS_INFO_NAMED("navigation_controller", "[navigation_controller %d]: Move to next subgoal", __LINE__);
				
				poseStampedSubGoal.clear();
				currGoal=0;
				//~ while (poseStampedPlan[currGoal].pose.position.x == mapAttributes.robotPose.dx+START_X && poseStampedPlan[currGoal].pose.position.y == mapAttributes.robotPose.dy+START_Y){
				while ((poseStampedPlan[currGoal].pose.position.x == mapAttributes.robotPose.dx+START_X && poseStampedPlan[currGoal].pose.position.y == mapAttributes.robotPose.dy+START_Y)  ||  pixelGoal.computeDistanceFrom(PixelCoords(mapAttributes.robotPose.dx+START_X, mapAttributes.robotPose.dy+START_Y)) <= DISTANCE_TO_APPROACH){
					currGoal++;
					
					//TODO: maybe this needs to go
					if ( currGoal > poseStampedPlan.size()-1 ){
						//~ return true;
						return 1;
					}
					
					pixelGoal=transformPoseStamped2PixelCoords(poseStampedPlan[currGoal]);
				}
				//~ ROS_INFO("navigation_controller","[NAVIGATION CONTROLLER %d]: robotPose  x=%d y=%d",__LINE__,mapAttributes.robotPose.dx+START_X,mapAttributes.robotPose.dy+START_Y);
				//~ ROS_INFO("[NAVIGATION CONTROLLER %d]: goal position x=%f y=%f",__LINE__,poseStampedPlan[currGoal].pose.position.x,poseStampedPlan[currGoal].pose.position.y);
				//~ ROS_INFO("[NAVIGATION CONTROLLER %d]: curr goal=%d DISTANCE_TO_APPROACH=%d",__LINE__,currGoal,DISTANCE_TO_APPROACH);
				poseStampedSubGoal.push_back(poseStampedPlan[currGoal]);
				navigator.setPlan( poseStampedSubGoal );
			}
		}
		//TODO: implement timer condition
		else if ( timeOutCounter == TIME_OUT_THRESHOLD ){
			//~ ROS_ERROR_NAMED("navigation_controller", "[navigation_controller %d]: Time out", __LINE__);
			stopMotors();
			//~ return false;
			return 2;
		}
		timeOutCounter++;
		
		//~ ROS_INFO_NAMED("navigation_controller_velocities", "[navigation_controller %d]: Requesting velocities", __LINE__);
		
		if ( !navigator.computeVelocityCommands(twistVelocities) ){
			//something went wrong with the navigator
			//~ ROS_ERROR_NAMED("navigation_controller", "[navigation_controller %d]: Error requesting velocities. STOPPING MOTORS", __LINE__);
			twistVelocities.linear.x =0;
			twistVelocities.angular.z =0;
			//~ stopMotors();
			//~ return false;
		}
		
		if ( checkPreempt() ){
			//~ ROS_ERROR_NAMED("navigation_controller", "[navigation_controller %d]: Navigation Preempted. STOPPING MOTORS", __LINE__);
			stopMotors();
			//~ return false;
			return 3;
		}
		
		sendSpeedsToMotors(twistVelocities);
		usleep(VELOCITIES_REQUEST_INTERVAL);
	}
	
	stopMotors();
	//~ return true;
	return 1;
}


int NavigationController::moveToTargetRecalculate_stuckAvoidance( const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& poseStampedPlan){
	int timeOutCounter = 0;
	
	float expRotational,expLinear;
	geometry_msgs::Twist twistVelocities;
	unsigned int currGoal=0;
	geometry_msgs::PoseStamped start;
	std::vector<geometry_msgs::PoseStamped> poseStampedSubGoal;
	poseStampedSubGoal.clear();
	PixelCoords pixelGoal=transformPoseStamped2PixelCoords(poseStampedPlan[currGoal]);
	
	ROS_INFO_NAMED("navigation_controller", "[navigation_controller %d]: Selecting first subgoal", __LINE__);
	
	if (stuckReset){
		ROS_ERROR_NAMED("robot_stuck", "[navigation_controller %d]: reseting stuck conditions", __LINE__);
		expectedRobotPose.dx = mapAttributes.robotPose.dx;
		expectedRobotPose.dy = mapAttributes.robotPose.dy;
		expectedRobotPose.theta = mapAttributes.robotPose.theta;
		expectedDx = 0;
		expectedDy = 0;
		expectedDtheta = 0;
		stuckCounter = 0;
		stuckReset = false;
	}
	
	
	
	while ( (poseStampedPlan[currGoal].pose.position.x == mapAttributes.robotPose.dx+START_X && poseStampedPlan[currGoal].pose.position.y == mapAttributes.robotPose.dy+START_Y  ) ||  pixelGoal.computeDistanceFrom(PixelCoords(mapAttributes.robotPose.dx+START_X, mapAttributes.robotPose.dy+START_Y)) <= DISTANCE_TO_APPROACH ){
		currGoal++;
		
		//TODO: maybe this needs to go
		if ( currGoal > poseStampedPlan.size()-1 ){
			//~ ROS_ERROR_NAMED("navigation_controller", "[navigation_controller %d]: TOO CLOSE TO TARGET. considered visited", __LINE__);
			return 1;
		}
		
		pixelGoal=transformPoseStamped2PixelCoords(poseStampedPlan[currGoal]);
	}
	poseStampedSubGoal.push_back(poseStampedPlan[currGoal]);
	//~ start = poseStampedPlan[currGoal];
	#ifdef FLOW
		//~ ROS_INFO("[NAVIGATION CONTROLLER %d]: robotPose  x=%d y=%d",__LINE__,mapAttributes.robotPose.dx+START_X,mapAttributes.robotPose.dy+START_Y);
		//~ ROS_INFO("[NAVIGATION CONTROLLER %d]: goal position x=%f y=%f",__LINE__,poseStampedPlan[currGoal].pose.position.x,poseStampedPlan[currGoal].pose.position.y);
		//~ ROS_INFO("[NAVIGATION CONTROLLER %d]: curr goal=%d DISTANCE_TO_APPROACH=%d",__LINE__,currGoal,DISTANCE_TO_APPROACH);
	#endif
	navigator.setPlan( poseStampedSubGoal );
	
	while (1){
		if ( navigator.isGoalReached() ){
			//reset timeout counter
			timeOutCounter = 0;
			
			stuckReset = false;
			
			
			ROS_INFO_NAMED("navigation_controller", "[navigation_controller %d]: Subgoal Reached", __LINE__);
			
			 
			if ( poseStampedPlan[currGoal].pose.position.x==goal.pose.position.x && poseStampedPlan[currGoal].pose.position.y==goal.pose.position.y ){//final goal reached. Success
				
				ROS_INFO_NAMED("navigation_controller", "[navigation_controller %d]: Final Goal reached", __LINE__);
				break; 
			}
			
			//recalculate path
			ROS_INFO_NAMED("navigation_controller", "[navigation_controller %d]: Recalculating path", __LINE__);
			if ( !pandoraPathPlanner.makePlan( start, goal, poseStampedPlan) ){//could NOT recalculate path!!!!
			
				ROS_ERROR_NAMED("navigation_controller", "[navigation_controller %d]: Could NOT recalculate path", __LINE__);
				stopMotors();
				//~ return false;
				return 2;
			}
			else{ //~ path recalculated. new subGoal found
				//TODO: reinitialize timer
				ROS_INFO_NAMED("navigation_controller", "[navigation_controller %d]: Move to next subgoal", __LINE__);
				
				poseStampedSubGoal.clear();
				currGoal=0;
				//~ while (poseStampedPlan[currGoal].pose.position.x == mapAttributes.robotPose.dx+START_X && poseStampedPlan[currGoal].pose.position.y == mapAttributes.robotPose.dy+START_Y){
				while ((poseStampedPlan[currGoal].pose.position.x == mapAttributes.robotPose.dx+START_X && poseStampedPlan[currGoal].pose.position.y == mapAttributes.robotPose.dy+START_Y)  ||  pixelGoal.computeDistanceFrom(PixelCoords(mapAttributes.robotPose.dx+START_X, mapAttributes.robotPose.dy+START_Y)) <= DISTANCE_TO_APPROACH){
					currGoal++;
					
					//TODO: maybe this needs to go
					if ( currGoal > poseStampedPlan.size()-1 ){
						//~ return true;
						return 1;
					}
					
					pixelGoal=transformPoseStamped2PixelCoords(poseStampedPlan[currGoal]);
				}
				//~ ROS_INFO("navigation_controller","[NAVIGATION CONTROLLER %d]: robotPose  x=%d y=%d",__LINE__,mapAttributes.robotPose.dx+START_X,mapAttributes.robotPose.dy+START_Y);
				//~ ROS_INFO("[NAVIGATION CONTROLLER %d]: goal position x=%f y=%f",__LINE__,poseStampedPlan[currGoal].pose.position.x,poseStampedPlan[currGoal].pose.position.y);
				//~ ROS_INFO("[NAVIGATION CONTROLLER %d]: curr goal=%d DISTANCE_TO_APPROACH=%d",__LINE__,currGoal,DISTANCE_TO_APPROACH);
				poseStampedSubGoal.push_back(poseStampedPlan[currGoal]);
				navigator.setPlan( poseStampedSubGoal );
			}
		}
		//TODO: implement timer condition
		else if ( timeOutCounter == TIME_OUT_THRESHOLD ){
			//~ ROS_ERROR_NAMED("navigation_controller", "[navigation_controller %d]: Time out", __LINE__);
			stopMotors();
			//~ return false;
			return 2;
		}
		timeOutCounter++;
		
		//~ ROS_INFO_NAMED("navigation_controller_velocities", "[navigation_controller %d]: Requesting velocities", __LINE__);
		
		if ( !navigator.computeVelocityCommands(twistVelocities) ){
			//something went wrong with the navigator
			//~ ROS_ERROR_NAMED("navigation_controller", "[navigation_controller %d]: Error requesting velocities. STOPPING MOTORS", __LINE__);
			twistVelocities.linear.x =0;
			twistVelocities.angular.z =0;
			//~ stopMotors();
			//~ return false;
		}
		
		if ( checkPreempt() ){
			//~ ROS_ERROR_NAMED("navigation_controller", "[navigation_controller %d]: Navigation Preempted. STOPPING MOTORS", __LINE__);
			stopMotors();
			//~ return false;
			return 3;
		}
		
		sendSpeedsToMotors(twistVelocities);
		/*
		//reset robot stuck conditions 
		if (stuckReset){
			ROS_WARN_NAMED("robot_stuck", "[navigation_controller %d]: reseting stuck conditions", __LINE__);
			expectedRobotPose.dx = mapAttributes.robotPose.dx;
			expectedRobotPose.dy = mapAttributes.robotPose.dy;
			expectedRobotPose.theta = mapAttributes.robotPose.theta;
			expectedDx = 0;
			expectedDy = 0;
			expectedDtheta = 0;
			stuckCounter = 0;
			stuckReset = false;
		}
		
		//TODO: these velocities need to change according to kinematic formulas!!!
		expRotational = fabs(twistVelocities.angular.z)<0.0001 ? 0: twistVelocities.angular.z;
		//~ expRotational = fabs(twistVelocities.angular.z/2)<0.0001 ? 0: twistVelocities.angular.z/2;
		expLinear = twistVelocities.linear.x /3;
		ROS_INFO_NAMED("robot_stuck", "[navigation_controller]: expLinear = %f expRotational = %f" , expLinear , expRotational);
		if (expRotational){
			//~ expectedRobotPose.dx += int( (-expLinear/expRotational*sin(expectedRobotPose.theta) + expLinear/expRotational*sin(expectedRobotPose.theta + expRotational*(VELOCITIES_REQUEST_INTERVAL/1000000))) / OCGD);
			//~ expectedRobotPose.dy += int( (expLinear/expRotational*cos(expectedRobotPose.theta) - expLinear/expRotational*cos(expectedRobotPose.theta + expRotational*(VELOCITIES_REQUEST_INTERVAL/1000000))) / OCGD);
			//~ expectedRobotPose.theta += expRotational*(VELOCITIES_REQUEST_INTERVAL/1000000);
			expectedDx += (-expLinear/expRotational*sin(expectedRobotPose.theta) + expLinear/expRotational*sin(expectedRobotPose.theta + expRotational*(0.1)));
			expectedDy += (expLinear/expRotational*cos(expectedRobotPose.theta) - expLinear/expRotational*cos(expectedRobotPose.theta + expRotational*(0.1)));
			expectedDtheta += expRotational*(0.1);
			
			ROS_INFO_NAMED("robot_stuck", "[navigation_controller]: x axis motion = %f" , (-expLinear/expRotational*sin(expectedRobotPose.theta) + expLinear/expRotational*sin(expectedRobotPose.theta + expRotational*(0.1))) );
			ROS_INFO_NAMED("robot_stuck", "[navigation_controller]: y axis motion = %f" , (expLinear/expRotational*cos(expectedRobotPose.theta) - expLinear/expRotational*cos(expectedRobotPose.theta + expRotational*(0.1))) );
			ROS_INFO_NAMED("robot_stuck", "[navigation_controller]: theta motion = %f" , expRotational*(0.1));
		}
		else{
			//~ expectedRobotPose.dx += int((expLinear * (VELOCITIES_REQUEST_INTERVAL/1000000)) * cos(expectedRobotPose.theta) / OCGD);
			//~ expectedRobotPose.dy += int((expLinear * (VELOCITIES_REQUEST_INTERVAL/1000000)) * sin(expectedRobotPose.theta) / OCGD);
			expectedDx += (expLinear * (0.1)) * cos(expectedRobotPose.theta);
			expectedDy += (expLinear * (0.1)) * sin(expectedRobotPose.theta);
			ROS_INFO_NAMED("robot_stuck", "[navigation_controller]: x axis motion = %f" , (expLinear * (0.1)) * cos(expectedRobotPose.theta) );
			ROS_INFO_NAMED("robot_stuck", "[navigation_controller]: y axis motion = %f" , (expLinear * (0.1)) * sin(expectedRobotPose.theta));
		}
		stuckCounter++;
		
		if ( stuckCounter == 20 ){
			ROS_ERROR_NAMED("robot_stuck", "[navigation_controller]: Updating expectedRobotPose");
			stuckCounter = 0;
			expectedRobotPose.dx += int(expectedDx / OCGD);
			expectedRobotPose.dy += int(expectedDy / OCGD);
			expectedRobotPose.theta += expectedDtheta;
			expectedDx = 0;
			expectedDy = 0;
			expectedDtheta = 0;
			
			ROS_ERROR_NAMED("robot_stuck", "[navigation_controller]: XDIFF = %f  ydiff = %f  thetadiff = %f", fabs(expectedRobotPose.dx - mapAttributes.robotPose.dx),fabs(expectedRobotPose.dy - mapAttributes.robotPose.dy), fabs(expectedRobotPose.theta - mapAttributes.robotPose.theta));
			ROS_ERROR_NAMED("robot_stuck", "[navigation_controller]: expectedRobotPose.dx = %d  expectedRobotPose.dy = %d  expectedRobotPose.theta = %f", expectedRobotPose.dx, expectedRobotPose.dy, expectedRobotPose.theta );
		}
		
		
		//~ ROS_ERROR_NAMED("robot_stuck", "[navigation_controller]: XDIFF = %f  ydiff = %f  thetadiff = %f", fabs(expectedRobotPose.dx - mapAttributes.robotPose.dx),fabs(expectedRobotPose.dy - mapAttributes.robotPose.dy), fabs(expectedRobotPose.theta - mapAttributes.robotPose.theta));
		//~ ROS_ERROR_NAMED("robot_stuck", "[navigation_controller]: expectedRobotPose.dx = %d  expectedRobotPose.dy = %d  expectedRobotPose.theta = %f", expectedRobotPose.dx, expectedRobotPose.dy, expectedRobotPose.theta );
		
		
		if ( fabs(expectedRobotPose.dx - mapAttributes.robotPose.dx) > EXPECTED_X_THRES || fabs(expectedRobotPose.dy - mapAttributes.robotPose.dy) > EXPECTED_Y_THRES || fabs(expectedRobotPose.theta - mapAttributes.robotPose.theta) > EXPECTED_THETA_THRES ){
			ROS_ERROR_NAMED("robot_stuck", "[navigation_controller %d]: Navigation to target paused to unstuck robot", __LINE__);
			ROS_ERROR_NAMED("robot_stuck", "[navigation_controller]: XDIFF = %f  ydiff = %f  thetadiff = %f", fabs(expectedRobotPose.dx - mapAttributes.robotPose.dx),fabs(expectedRobotPose.dy - mapAttributes.robotPose.dy), fabs(expectedRobotPose.theta - mapAttributes.robotPose.theta));
			stopMotors();
			#ifdef SIMPLE_UNSTUCK_ROTATION
				rotateRobotToUnstuck();
			#elif defined UNSTUCK_ROTATION_WITH_CORRECTIONS
				moveRobotToUnstuck();
			#endif
			stuckReset = true;
			continue;
		}
		*/
		usleep(VELOCITIES_REQUEST_INTERVAL);
	}
	
	stopMotors();
	//~ return true;
	return 1;
}




void NavigationController::sendSpeedsToMotors( geometry_msgs::Twist twistVelocities){
	//~ twistVelocities.linear.x = -twistVelocities.linear.x ;
	//~ twistVelocities.angular.z = -twistVelocities.angular.z ;
	_velocityPublisher.publish(twistVelocities);
	
	//~ main_motor_control_communications::setVehicleSpeedGoal _setVehicleSpeedGoal;
	//~ float linearSpeed=0, rotationalSpeed=0;
	//~ 
	//~ transformTwist2Velocities(twistVelocities, &linearSpeed, &rotationalSpeed);
	//~ 
	//~ _setVehicleSpeedGoal.linearSpeed = linearSpeed;
	//~ _setVehicleSpeedGoal.rotationalSpeed = -rotationalSpeed; //TODO:move minus into set speeds.
	//~
	
}


void NavigationController::stopMotors(void){
	ROS_WARN_NAMED("navigation_controller", "[navigation_controller %d]: Stopping Motors", __LINE__);
	geometry_msgs::Twist veloc;
	veloc.linear.x =0;
	veloc.angular.z =0;
	sendSpeedsToMotors(veloc);
	
}

PixelCoords NavigationController::transformPoseStamped2PixelCoords(geometry_msgs::PoseStamped poseStamped){
	
	int x = poseStamped.pose.position.x;
	int y = poseStamped.pose.position.y;
	
	return PixelCoords(x,y);
	
}


geometry_msgs::PoseStamped NavigationController::transformPixelCoords2PoseStamped(PixelCoords pixelCoords){
	
	float x = pixelCoords.getXCoord();
	float y = pixelCoords.getYCoord();
	
	geometry_msgs::PoseStamped poseStamped;
	
	poseStamped.pose.position.x = x;
	poseStamped.pose.position.y = y;
	
	return poseStamped;
	
	
}


void NavigationController::transformTwist2Velocities( geometry_msgs::Twist twist, float* linear, float* rotational){
	
	*linear = twist.linear.x ;
	*rotational = twist.angular.z ;
	
}




/**
	@brief Serves the laser scan messages
	@param msg [const slam_communications::laserScanMsg&] : The laser scan message
	@return void
	**/
void NavigationController::updateLaserScans(const sensor_msgs::LaserScanConstPtr& msg){
	
	lsr.sLASER_RAYS=msg->ranges.size();
	lsr.sD_LASER_MAX=msg->range_max;
	lsr.sD_LASER_ANG=(msg->angle_max-msg->angle_min)*180/D_PI;
	lsr.sD_LASER_ANG_RAD=msg->angle_max-msg->angle_min;
	lsr.sD_LASER_ANGLE_BEGIN=msg->angle_min;
	lsr.sD_LASER_ANGLE_END=msg->angle_max;
	lsr.sD_LASER_COEFF_FIX=1;	
	
	//~ ROS_INFO_NAMED("navigation_controller_laser", "[navigation_controller %d]: Updating laser scans ", __LINE__);
	for(unsigned int i=0;i<msg->ranges.size();i++){
			scan.scan[i]=msg->ranges[i];
			//~ ROS_ERROR("scan.scan[i] %f ",scan.scan[i]);
	}

	//~ float lc=linearCorrection();
	//~ float rc=rotationalCorrection();

}

void NavigationController::serveCompassMessage(const controllers_and_sensors_communications::compassMsg& msg){
	compassPitch=msg.pitch;
	//~ ROS_INFO_NAMED("navigation_controller_compass", "[navigation_controller %d]: Compass : %f ", __LINE__,compassPitch);
}


/**
	@brief Serves the sonar scan messages
	@param msg [const controllers_and_sensors_communications::sonarMsg&] : The sonar scan message
	@return void
	**/
void NavigationController::updateSonars(const sensor_msgs::Range& msg){

	if(msg.header.frame_id=="left_rear_sonar")
		sonars.backLeft=msg.range;
	else if(msg.header.frame_id=="right_rear_sonar")
		sonars.backRight=msg.range;
	else if(msg.header.frame_id=="head_sonar")
		sonars.arm=msg.range;
}


bool NavigationController::sendMap(pandora_navigation_communications::navigationMapSrv::Request &req,pandora_navigation_communications::navigationMapSrv::Response &res){
	
	if(!_mapUpdater.slamInitialized) return false;
	
	int prevxMax = mapAttributes.prevxMax;
	int prevyMax = mapAttributes.prevyMax;
	int prevxMin = mapAttributes.prevxMin;
	int prevyMin = mapAttributes.prevyMin;

	//--------------------- map ---------------------//
	res.xsize=prevxMax-prevxMin;
	res.ysize=prevyMax-prevyMin;

	//--------------------- partition graph voronoi ---------------------//
	res.voronoi=std::vector<unsigned char>((prevxMax-prevxMin)*(prevyMax-prevyMin));
	for(int i=0;i<(prevxMax-prevxMin);i++)
		for(int j=0;j<(prevyMax-prevyMin);j++)
			res.voronoi[i*(prevyMax-prevyMin)+j]=pandoraPathPlanner.partitionGraphPathGenerator.voronoi->voronoi[i+prevxMin][j+prevyMin];

	//--------------------- partition graph ---------------------//
	res.decompx=std::vector<int>(pandoraPathPlanner.partitionGraphPathGenerator.partitionNodes.nodes.size());
	res.decompy=std::vector<int>(pandoraPathPlanner.partitionGraphPathGenerator.partitionNodes.nodes.size());
	for(unsigned int i=0;i<pandoraPathPlanner.partitionGraphPathGenerator.partitionNodes.nodes.size();i++){
		res.decompx[i]=pandoraPathPlanner.partitionGraphPathGenerator.partitionNodes.nodes[i].p.getXCoord()-prevxMin;
		res.decompy[i]=pandoraPathPlanner.partitionGraphPathGenerator.partitionNodes.nodes[i].p.getYCoord()-prevyMin;
	}
	
	res.decompNeighborsFirst.clear();
	res.decompNeighborsLast.clear();
	for(unsigned int i=0;i<pandoraPathPlanner.partitionGraphPathGenerator.partitionNodes.nodes.size();i++){
		for(unsigned int j=0;j<pandoraPathPlanner.partitionGraphPathGenerator.partitionNodes.nodes[i].neighID.size();j++){
			res.decompNeighborsFirst.push_back(pandoraPathPlanner.partitionGraphPathGenerator.partitionNodes.nodes[i].ID);
			res.decompNeighborsLast.push_back(pandoraPathPlanner.partitionGraphPathGenerator.partitionNodes.nodes[i].neighID[j]);
		}
	}

	return true;
}


bool NavigationController::checkPreempt(){
		if(moveBaseActionServer_.isPreemptRequested()==true || !ros::ok()){
			return true;
		}
		return false;
}

void NavigationController::publishPath(const ros::TimerEvent&){
	
	nav_msgs::Path pathForViz;
	geometry_msgs::PoseStamped pathPoint;
	
	for (int i=0; i<poseStampedPlan.size();i++){
	
		pathPoint.pose.position.x = ( - START_Y + poseStampedPlan[i].pose.position.x)*0.02;
		pathPoint.pose.position.y = ( - START_X + poseStampedPlan[i].pose.position.y)*0.02;
		pathForViz.poses.push_back(pathPoint);
	}
	
	pathForViz.header.stamp = ros::Time::now();
	pathForViz.header.frame_id = "world";

	_pathPublisher.publish(pathForViz);
	
}





