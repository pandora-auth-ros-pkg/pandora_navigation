#include "ros/ros.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <pandora_navigation_communications/InitialTurnAction.h>
#include <target_selector_communications/SelectTargetAction.h>
#include <geometry_msgs/PoseStamped.h>

#include <iostream>
#include <actionlib/client/simple_action_client.h>

int main(int argc,char** argv){
	
	ros::init(argc,argv,"dummy_fsm");

	//~ actionlib::SimpleActionClient<pandora_navigation_communications::MoveBaseAction> moveBaseActionClient =
		//~ actionlib::SimpleActionClient<pandora_navigation_communications::MoveBaseAction>("/move_base",true);
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> moveBaseActionClient("/move_base",true);
	actionlib::SimpleActionClient<pandora_navigation_communications::InitialTurnAction> initialTurnActionClient("/initial_turn",true);
	
	actionlib::SimpleActionClient<target_selector_communications::SelectTargetAction> selectTargetActionClient("/select_target",true);
	
	target_selector_communications::SelectTargetGoal targetSelectionGoal ;
	targetSelectionGoal.targetType=target_selector_communications::SelectTargetGoal::TYPE_EXPLORATION;
	
	
	move_base_msgs::MoveBaseGoal goal ;
	
	//perform initial turn
	printf("Performing Initial Turn\n");
	pandora_navigation_communications::InitialTurnGoal initialGoal ;
	initialTurnActionClient.waitForServer();
	initialTurnActionClient.sendGoal(initialGoal);
	initialTurnActionClient.waitForResult();
	
	
	//~ goal.target_pose.pose.position.x =  2048 ; 
	//~ goal.target_pose.pose.position.y =  2048 ;
	//~ moveBaseActionClient.waitForServer();
	//~ moveBaseActionClient.sendGoal(goal);
	//~ printf("forwarded goal(initial turn)\n");
	//~ moveBaseActionClient.waitForResult();
	
	
	while(1){
	
		printf("Asking for target \n");
		
		selectTargetActionClient.waitForServer();
		selectTargetActionClient.sendGoal(targetSelectionGoal);
		selectTargetActionClient.waitForResult();
	
		geometry_msgs::PoseStamped pose_stamped  = selectTargetActionClient.getResult()->target_pose;

		printf("Received target : x= %f , y = %f \n",pose_stamped.pose.position.x ,pose_stamped.pose.position.y );

		//~ pose_stamped.pose.position.x =  atoi(argv[1]) + 2048 ; 
		//~ pose_stamped.pose.position.y =  atoi(argv[2]) + 2048 ;
		//~ pose_stamped.pose.position.z = 0  ;
	//~ 
		//~ pandora_navigation_communications::MoveBaseGoal goal ;
		
		//~ goal.target_pose = pose_stamped;
		
		goal.target_pose = pose_stamped;
			
		moveBaseActionClient.waitForServer();
		moveBaseActionClient.sendGoal(goal);
		printf("forwarded goal\n");
		moveBaseActionClient.waitForResult();
	}
	
}
