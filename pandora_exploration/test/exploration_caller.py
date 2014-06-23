#! /usr/bin/env python

import roslib
import rospy
import sys
import actionlib
from pandora_navigation_msgs.msg import DoExplorationAction, DoExplorationGoal

if __name__ == '__main__':
  rospy.init_node('exploration_caller')
  client = actionlib.SimpleActionClient('do_exploration', DoExplorationAction)
  client.wait_for_server()
  goal = DoExplorationGoal()
  client.send_goal(goal)
  client.wait_for_result()
  rospy.logwarn(client.get_goal_status_text())
