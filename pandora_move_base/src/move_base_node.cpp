
#include "pandora_move_base/navigation_controller.h"
#include "ros/ros.h"

int main(int argc, char **argv){
	
	ros::init(argc,argv,"move_base_node",ros::init_options::NoSigintHandler);
	NavigationController moveBitch;
	ROS_INFO("[MOVE BASE NODE]: ROS spin");
	ros::spin();
	
	return 0;
}
