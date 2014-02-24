#include "ros/ros.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>

#include <iostream>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

int main(int argc,char** argv){
	
	ros::init(argc,argv,"move_base_client");

	//~ actionlib::SimpleActionClient<pandora_navigation_communications::MoveBaseAction> moveBaseActionClient =
		//~ actionlib::SimpleActionClient<pandora_navigation_communications::MoveBaseAction>("/move_base",true);
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> moveBaseActionClient("/move_base",true);
	
	geometry_msgs::PoseStamped pose_stamped ;
	
	ros::NodeHandle _nh;
	
	tf::TransformListener tfListener;
	tf::StampedTransform robotPoseTransform;
	
	if ( !tfListener.waitForTransform ("/robotCenter", "/world", ros::Time(0), ros::Duration(5)) ){
		ROS_DEBUG("WARNING(@dummy_fsm) transform is not being published " );
		return false;
	}
	try{
		tfListener.lookupTransform("/robotCenter", "/world",
		ros::Time(0), robotPoseTransform);
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
	}
	
	tf::Vector3 robotPoseVector=robotPoseTransform.getOrigin();
	
	int currentX = robotPoseVector[0]*50;
	int currentY = robotPoseVector[1]*50;
	
	pose_stamped.pose.position.x =  atoi(argv[1]) + currentX ; 
	pose_stamped.pose.position.y =  atoi(argv[2]) + currentY ;
	pose_stamped.pose.position.z = 0  ;

	move_base_msgs::MoveBaseGoal goal ;
	
	goal.target_pose = pose_stamped;
		
	moveBaseActionClient.waitForServer();
	moveBaseActionClient.sendGoal(goal);
	printf("forwarded goal\n");
	
}
