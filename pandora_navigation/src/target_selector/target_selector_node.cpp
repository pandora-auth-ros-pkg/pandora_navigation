
//~ #include "target_selector/target_selector.h"
#include "target_selector/target_selector_controller.h"
//~ #include "target_selector/closest_unexplored_target_selector.h"
#include "ros/ros.h"

int main(int argc, char **argv){
	
	ros::init(argc,argv,"target_selector_node",ros::init_options::NoSigintHandler);
	TargetSelectorController kitsos;
	ros::spin();
	
	return 0;
}
