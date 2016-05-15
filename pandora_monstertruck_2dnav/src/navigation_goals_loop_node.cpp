/*********************************************************************
*
* software license agreement (bsd license)
*
*  copyright (c) 2016, p.a.n.d.o.r.a. team.
*  all rights reserved.
*
*  redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * neither the name of the p.a.n.d.o.r.a. team nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  this software is provided by the copyright holders and contributors
*  "as is" and any express or implied warranties, including, but not
*  limited to, the implied warranties of merchantability and fitness
*  for a particular purpose are disclaimed. in no event shall the
*  copyright owner or contributors be liable for any direct, indirect,
*  incidental, special, exemplary, or consequential damages (including,
*  but not limited to, procurement of substitute goods or services;
*  loss of use, data, or profits; or business interruption) however
*  caused and on any theory of liability, whether in contract, strict
*  liability, or tort (including negligence or otherwise) arising in
*  any way out of the use of this software, even if advised of the
*  possibility of such damage.
*
* author:  George Kouros
*********************************************************************/

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
  MoveBaseClient;

int main(int argc, char** argv)
{
  // initialize node
  std::string nodeName = "navigation_goals_loop_node";
  ros::init(argc, argv, nodeName);
  ros::NodeHandle nh(nodeName);

  XmlRpc::XmlRpcValue goalList;

  // load goals from param server
  if ( !nh.getParam("goals", goalList) )
  {
    ROS_ERROR("Failed to load navigation goals! Exiting...");
    exit(-1);
  }
  else
  {
    ROS_ASSERT(goalList.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for (int ii = 0; ii < goalList.size(); ii++)
    {
      ROS_ASSERT(goalList[ii].getType() == XmlRpc::XmlRpcValue::TypeArray);

      ROS_ASSERT(goalList[ii].size() == 4);

      for (int jj = 0; jj < 4; jj++)
        ROS_ASSERT(goalList[ii][jj].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    }

    ROS_INFO("Navigation Goals loaded successfully!");
  }

  // tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0)))
    ROS_INFO("Waiting for move_base action server to come up");


  while (ros::ok())
  {
    for (int ii = 0; ii < goalList.size(); ii++)
    {
      move_base_msgs::MoveBaseGoal goal;
      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp = ros::Time::now();
      goal.target_pose.pose.position.x = static_cast<double>(goalList[ii][0]);
      goal.target_pose.pose.position.y = static_cast<double>(goalList[ii][1]);
      goal.target_pose.pose.orientation.z = static_cast<double>(goalList[ii][2]);
      goal.target_pose.pose.orientation.w = static_cast<double>(goalList[ii][3]);

      ROS_INFO_STREAM("Sending goal" << goalList[ii]);

      ac.sendGoal(goal);

      ac.waitForResult();

      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO_STREAM("Goal " << goalList[ii] << " succeeded!");
      else
      {
        ROS_INFO_STREAM("Goal " << goalList[ii] << " failed!");
        ii--;  // repeat goal until success
      }

    }
  }

  return 0;
}
