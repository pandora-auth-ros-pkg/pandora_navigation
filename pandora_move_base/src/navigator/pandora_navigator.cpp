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

#include "pandora_move_base/navigator/pandora_navigator.h"

//~ PandoraNavigator::PandoraNavigator(int argc, char **argv){
PandoraNavigator::PandoraNavigator(Transformation& _robotPose, LaserScan& _scan, SonarScan& _sonars, IrScan& _irs, float& _compassPitch,const LaserInfo& lsr_): robotPose(_robotPose), scan(_scan), sonars(_sonars), irs(_irs), compassPitch(_compassPitch),lsr(lsr_){
	
	
	//~ robotPose.dx=0;
	//~ robotPose.dy=0;
	//~ robotPose.theta=0;
	float laserrad=0;
	int laserrays=0;
	
	if(lsr.sLASER_RAYS==0){
		laserrays=640;
		laserrad=2.268890*2;
	}else{
		laserrays=lsr.sLASER_RAYS;
		laserrad=lsr.sD_LASER_ANG_RAD;
	}
	
	cosTheta=new float[laserrays];
	sinTheta=new float[laserrays];
	
	for(unsigned int i=0;i<laserrays;i++){
		float angle=((float)i/laserrays-0.5)*(laserrad);
		cosTheta[i]=cos(angle);
		sinTheta[i]=sin(angle);
	}
	
	isRobotStarted=true;
	firstTime=0;
	//~ ROS_INFO("[NAVIGATOR %d] Leaving Constructor with robotpose.dx=%d robotpose.dy=%d robotpose.theta=%f",__LINE__,robotPose.dx,robotPose.dy,robotPose.theta);
	//~ pathSize=0;
	
}

float PandoraNavigator::calculateTurnCoefficient(PixelCoords rPose, PixelCoords sgPose,float rYaw){
	
	float theta_1,theta_2,temp_;
	theta_1=atan2(sgPose.getYCoord()-rPose.getYCoord(),sgPose.getXCoord()-rPose.getXCoord());
	theta_2=rYaw;
	temp_=theta_1-theta_2;
	float speed=0;
	if(abs(temp_)>D_PI && temp_>0)
		speed=-(2*D_PI-fabs(temp_));
	else if(abs(temp_)>D_PI && temp_<0)
		speed=(2*D_PI-fabs(temp_));
	else if(abs(temp_)<D_PI)
		speed=temp_;
	
	//~ ROS_ERROR("%d %d %d %d %f %f %f %f",rPose.getXCoord(),rPose.getYCoord(),sgPose.getXCoord(),sgPose.getYCoord(),rYaw,theta_1,temp_,speed/D_PI);
	
	return speed/D_PI;
}

//bool PandoraNavigator::moveToTarget( PixelCoords goal ){
bool PandoraNavigator::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
	
	ROS_INFO_NAMED("navigator","[navigator %d]: Computing velocities",__LINE__);
	
	//if subGoal is reached and multiple targets, move to next subgoal
	if ( multipleSubgoals && pixelPlan[currGoal].computeDistanceFrom(PixelCoords(robotPose.dx+START_X, robotPose.dy+START_Y)) <= DISTANCE_TO_APPROACH ){
		
		ROS_INFO_NAMED("navigator","[navigator %d]: Changing Goal",__LINE__);
		
		currGoal++;
		if ( currGoal == pixelPlan.size() || isGoalReached() ){ //final goal is reached
			
			cmd_vel = transformVelocities2Twist(0, 0);
			
			return true;
		}
	}

	if ( pixelPlan[currGoal].computeDistanceFrom(PixelCoords(robotPose.dx+START_X, robotPose.dy+START_Y)) > DISTANCE_TO_APPROACH ){
		
		
		float dist,robotposex,robotposey,goalx,goaly;
		float theta_1,theta_2,temp_;
		
		float rot1,rot2,rot3,rot4,lin1,lin2,lin3;
		
		linearSpeed=0;
		rotationalSpeed=0;
		
		//	Current robot pose
		robotposex=(float)robotPose.dx;
		robotposey=(float)robotPose.dy;

		goalx=(float)pixelPlan[currGoal].getXCoord()-START_X;
		goaly=(float)pixelPlan[currGoal].getYCoord()-START_Y;
		
		rot1=calculateTurnCoefficient(PixelCoords(robotposex,robotposey),PixelCoords(goalx,goaly),robotPose.theta);
		
		lin1=LINEAR_MAX*((1-sqrt(sqrt(fabs(rot1))))*(fabs(rot1)<0.2)+(0.01)*((fabs(rot1)>=0.2)));
		lin2=performLinearCorrection();
		lin3=lin1-lin1*sqrt(lin2);
		
		rot2=rot1*(ROTATIONAL_MAX);
		rot3=performRotationalCorrection();
		rot4=rot2-fabs(rot3)/(rot3+0.0000001)*pow(rot3,2);

		dist=(robotposey-goaly)*(robotposey-goaly)+(robotposex-goalx)*(robotposex-goalx);

		linearSpeed=lin3;
		rotationalSpeed=rot4;

		cmd_vel = transformVelocities2Twist(linearSpeed, rotationalSpeed);
		return true;
	}
	else{
		ROS_ERROR_NAMED("navigator","[navigator %d]: Speeds NOT found. Target too close to robot(visited)",__LINE__);
		//~ exit(1);
		return false;
	}
}


bool PandoraNavigator::isGoalReached(void){
	PixelCoords currPos=PixelCoords(robotPose.dx, robotPose.dy);
	
	ROS_INFO_NAMED("navigator_goal_reached","[navigator %d]:Checking if goal is reached",__LINE__);
	//TODO: check if start_position subtraction is needed
	return (currPos.computeDistanceFrom(PixelCoords(pixelPlan[pixelPlan.size()-1].getXCoord()-START_X, pixelPlan[pixelPlan.size()-1].getYCoord()-START_Y))<DISTANCE_TO_APPROACH);
	
	//return ( currPos.computeDistanceFrom(PixelCoords(target.getXCoord(), target.getYCoord())) < DISTANCE_TO_APPROACH );
	
	
}


 bool PandoraNavigator::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan){
	
	pixelPlan.clear();
	
	//TODO: edit to work with more than one subgoals
	for (unsigned int i=0; i< plan.size(); i++)
		pixelPlan.push_back( transformPoseStamped2PixelCoords(plan[i]) );
	//~ pixelPlan.push_back( transformPoseStamped2PixelCoords(plan[0]) );
	
	currGoal = 0;
	multipleSubgoals = (pixelPlan.size()>1)? true: false;
	//~ #ifdef FLOW
		//~ ROS_INFO("[NAVIGATOR %d]: Set plan  multipleSubgoals=%d",__LINE__,multipleSubgoals);
	//~ #endif
}






//~ void PandoraNavigator::performInitialTurn(void){
void PandoraNavigator::performInitialTurn(geometry_msgs::Twist& cmd_vel){
	
	//~ firstTime=0;
	
	//~ while(firstTime<35*2 && isRobotStarted){
	//~ while(firstTime<35*2){
		
		//~ ROS_INFO("[NAVIGATOR %d] Turning around.. #1",__LINE__);
		linearSpeed = performCircleCorrection();
		//ROS_ERROR("[NAVIGATOR] Circle correction is : %f", p->linearSpeed);
		//~ float compassCorrection=0.4*compassPitch;
		//ROS_ERROR("[NAVIGATOR] Correction from compass : %f",compassCorrection);
		//~ compassCorrection=linearSpeed+compassCorrection;
		//~ if(compassCorrection>LINEAR_MAX/2)
			//~ compassCorrection=LINEAR_MAX/2;
		//~ else if(compassCorrection<-LINEAR_MAX/2)
			//~ compassCorrection=-LINEAR_MAX/2;
		//~ linearSpeed=compassCorrection;
		//ROS_ERROR("[NAVIGATOR] Correction Final : %f",p->linearSpeed);
		
		rotationalSpeed = 0.3;
		
		//~ if(fabs((compassPitch)>=0.03))
			//~ rotationalSpeed=0.15;
		//~ else 
			//~ rotationalSpeed=0.2;
		//~ firstTime++;
		//~ usleep(500000);
		
		cmd_vel = transformVelocities2Twist(linearSpeed*0.45, rotationalSpeed);
		
		ROS_INFO_NAMED("navigator_velocities","[navigator %d]: Initial turn speeds found. linearSpeed=%f  rotationalSpeed=%f", __LINE__,linearSpeed*0.45,rotationalSpeed);
		
}


void PandoraNavigator::parkRobot( geometry_msgs::Twist& cmd_vel, float angleToTurn ){
	
	float rotational=0.05*angleToTurn/fabs(angleToTurn);
	//~ float rotational=0.10*angleToTurn/fabs(angleToTurn);
	
	float linear=performCircleCorrection();
	float compassCorrection=0.4*compassPitch;
	compassCorrection=linear+compassCorrection;
	if(compassCorrection>LINEAR_MAX/2)
		compassCorrection=LINEAR_MAX/2;
	else if(compassCorrection<-LINEAR_MAX/2)
		compassCorrection=-LINEAR_MAX/2;
	linear=compassCorrection*0.2;
	
	cmd_vel = transformVelocities2Twist(linear, rotational);
	ROS_INFO_NAMED("navigator_velocities","[navigator %d]: Parking speeds found. linearSpeed=%f  rotationalSpeed=%f", __LINE__,linear,rotational);
}

float PandoraNavigator::performLinearCorrection(void){
	
	float headYaw, headPitch;
	
	getPitchAndYaw( &headPitch, &headYaw );

	float correction=0.0;
	float cossum=0.0;
	for(unsigned int i=0;i<lsr.sLASER_RAYS;i++){
		cossum+=pow(cosTheta[i],2);
		correction+=pow(cosTheta[i],2)/(1.0+pow((float)scan.scan[i],2));
	}
	correction/=cossum;	//This is from 0-1
	
	float sonarsCorr=1/(1+pow(sonars.backLeft/2+sonars.backRight/2,2));
	
	correction-=sonarsCorr;
	//	From [-1,1]
	return correction;	
}


/**
	@brief Function that performs obstacle avoidance while in a circle
	@param void
	@return The correction
	**/

//~ float PandoraNavigator::performCircleCorrection(void){
float PandoraNavigator::performCircleCorrection(bool stuck/*=false*/){
	float correction=0;
	float minDistance=INFINITY_DISTANCE;

	float minLaser=INFINITY_DISTANCE;
		
	for(unsigned int i=0;i<lsr.sLASER_RAYS;i++){
		if( (scan.scan[i]<minLaser && scan.scan[i]>0.01 ) ){
			minLaser=scan.scan[i];
		}
	}
	
	ROS_INFO_NAMED("navigator_corrections","[navigator %d]:  minDistance from laser is = %f",__LINE__,minLaser);
	
	
	float headYaw, headPitch;
	
	getPitchAndYaw( &headPitch, &headYaw );

	
	ROS_INFO_NAMED("navigator_corrections","[navigator %d]:  After arm sonar minDistance from laser is = %f",__LINE__,minLaser);
	
	if (stuck){
		
		ROS_INFO_NAMED("robot_stuck","[navigator %d]:  After arm sonar minDistance from laser is = %f",__LINE__,minLaser);
		correction=minLaser-0.1;
		
	}
	else{
		ROS_INFO_NAMED("navigator_corrections","[navigator %d]:  Getting back sonar corrections",__LINE__);
		if(minLaser<0.4 || sonars.backRight<0.4 || sonars.backLeft<0.4){ // sonars.left<0.5 || sonars.right<0.5
			if(sonars.backRight<minDistance)
				minDistance=sonars.backRight;
			if(sonars.backLeft<minDistance)
				minDistance=sonars.backLeft;
			//~ if(sonars.left<minDistance)
				//~ minDistance=sonars.left;
			//~ if(sonars.right<minDistance)
				//~ minDistance=sonars.right;
			
			
			correction=minLaser-minDistance;
			
		}
	}
		
	ROS_INFO_NAMED("navigator_corrections","[navigator %d]:  minDistance from sonars is = %f",__LINE__,minDistance);

	if(correction>LINEAR_MAX)
		correction=LINEAR_MAX;
	else if(correction<-LINEAR_MAX)
		correction=-LINEAR_MAX;
	
	ROS_INFO_NAMED("navigator_corrections","[navigator %d]: Circle Correction after= %f",__LINE__,correction);
	
	return correction;
}



/**
	@brief Function that performs obstacle avoidance by changing the rotational speed
	@param void
	@return The correction
	**/
float PandoraNavigator::performRotationalCorrection(void){
	float correction=0.0;
	float cossum=0.0;
	for(unsigned int i=0;i<lsr.sLASER_RAYS;i++){
		correction+=sinTheta[i]/(1.0+pow((float)scan.scan[i],2));
	}
	correction/=200.0;
	return correction;
}


float PandoraNavigator::rotateToUnstuck(void){
	float correction=0,correctionLaser=0,correctionSonar=0,correctionLines=0;
	for(unsigned int i=0;i<lsr.sLASER_RAYS;i++){
		if(scan.scan[i]!=0)
			correctionLaser-=sinTheta[i]/pow((float)scan.scan[i]-EXTRA_OBSTACLE, 2)/lsr.sD_LASER_MAX;
	}
	correctionLaser*=0.9;

	//~ correctionSonar-=(1.0/pow(sonars.backRight,3))*20;
	//~ correctionSonar+=(1.0/pow(sonars.backLeft,3))*20;
	//~ correctionSonar-=(1.0/pow(sonars.right,3));
	//~ correctionSonar+=(1.0/pow(sonars.left,3));
	
	//~ if((Timer::now()-lastSeenColor)<TIME_TO_LINES){
		//~ correctionLines=colorLineTheta*colorIntensity;
	//~ }
	
	//~ correctionSonar*=0.1;
	//~ correction=correctionLaser+correctionSonar; //+correctionLines
	
	//~ ROS_INFO("[NAV_CORRECTIONS] Rotational Correction from sonars : %f",correction*ROTATIONAL_COEF);
	
	//~ if (correctionSonar > correctionLaser){
		//~ linearSpeed += LINEAR_MAX/2;
	//~ }
	//~ else{
		//~ linearSpeed -= LINEAR_MAX/2;
	//~ }
	//~ linearSpeed += performLinearCorrection();
	
	if(correctionLaser>ROTATIONAL_MAX)
		correctionLaser=ROTATIONAL_MAX;
	else if(correctionLaser<-ROTATIONAL_MAX)
		correctionLaser=-ROTATIONAL_MAX;
	
	//~ return correctionLaser*ROTATIONAL_COEF;
	return correctionLaser;
}








void PandoraNavigator::getPitchAndYaw(float* pitchPtr,float* yawPtr)
{
	tf::StampedTransform tfTransform;
	
	try {
		
		//~ _listener.waitForTransform("robotCenter", "headCamera", ros::Time(0) , ros::Duration(1));
		
		//~ _listener.lookupTransform( "robotCenter", "headCamera", ros::Time(0) , tfTransform);
		
	}
	catch (tf::TransformException ex) {
		ROS_ERROR("[my_object %d]%s", __LINE__, ex.what());
	}
	
	tfScalar  roll, pitch, yaw ;
	
	tfTransform.getBasis().getRPY(roll, pitch, yaw);
	
	*pitchPtr =  pitch;
	*yawPtr =  yaw;
	
}




PixelCoords PandoraNavigator::transformPoseStamped2PixelCoords(geometry_msgs::PoseStamped poseStamped){
	
	int x = poseStamped.pose.position.x;
	int y = poseStamped.pose.position.y;
	
	return PixelCoords(x,y) ;
	
}


geometry_msgs::PoseStamped PandoraNavigator::transformPixelCoords2PoseStamped(PixelCoords pixelCoords){
	
	float x = pixelCoords.getXCoord();
	float y = pixelCoords.getYCoord();
	
	geometry_msgs::PoseStamped poseStamped;
	
	poseStamped.pose.position.x = x;
	poseStamped.pose.position.y = y;
	
	return poseStamped;
	
	
}

geometry_msgs::Twist PandoraNavigator::transformVelocities2Twist(float linear, float rotational){
	
	geometry_msgs::Twist twist;
	
	twist.linear.x =  linear;
	twist.angular.z =  rotational;
	
	return twist;
	
}









