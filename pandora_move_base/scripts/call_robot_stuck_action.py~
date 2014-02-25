#!/usr/bin/env python
import roslib; 
roslib.load_manifest('pandora_navigation')
import rospy
import actionlib
from pandora_navigation_communications.msg import *


if __name__ == '__main__':
    rospy.init_node('set_vehicle_speed_wrapper')
    client = actionlib.SimpleActionClient('/robot_stuck', RobotStuckAction)
    stuck_goal = RobotStuckGoal()
    client.wait_for_server()
    print "Sending robot_stuck goal"
    client.send_goal(stuck_goal)
    client.wait_for_result()
    rospy.spin()
	
