#include <ros/ros.h>
#include "pandora_exploration/frontier_goal_selector.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "frontier_node", ros::init_options::NoSigintHandler);
  pandora_exploration::FrontierGoalSelector frontier_selector;

  ros::Rate rate(0.5);

  while (ros::ok()) {
    
    ROS_INFO("Updating frontiers");
    geometry_msgs::PoseStamped goal;
    frontier_selector.findNextGoal(&goal);

    ros::spinOnce();
    
    rate.sleep();
  }
  
  return 0;
}



