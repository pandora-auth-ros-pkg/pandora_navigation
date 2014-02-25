#!/usr/bin/env python
import roslib; 
roslib.load_manifest('pandora_move_base')
import rospy
import actionlib
import pandora_navigation_communications.msg 
import geometry_msgs.msg 


class RvizMoveBaseWrapper(object):

    def __init__(self):
        self.sub = rospy.Subscriber("/move_base_simple/goal", geometry_msgs.msg.PoseStamped, self.execute_cb)
        self.client = actionlib.SimpleActionClient('/move_base', pandora_navigation_communications.msg.MoveBaseAction)
    
    def execute_cb(self, goal):
        forwarder_goal = pandora_navigation_communications.msg.MoveBaseGoal()
        forwarder_goal.target_pose = goal
        print "Got goal , forwarding"
        self.client.send_goal(forwarder_goal)
        #~ self.client.wait_for_result()

    

if __name__ == '__main__':
	rospy.init_node('set_vehicle_speed_wrapper')
	wrapper = RvizMoveBaseWrapper()
	rospy.spin()
	
