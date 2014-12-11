#!/usr/bin/env python
# Software License Agreement
__version__ = "0.0.1"
__status__ = "Production"
__license__ = "BSD"
__copyright__ = "Copyright (c) 2014, P.A.N.D.O.R.A. Team. All rights reserved."
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of P.A.N.D.O.R.A. Team nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
__author__ = "Tsirigotis Christos and Voulgarakis George"
__maintainer__ = "Tsirigotis Christos"
__email__ = "tsirif@gmail.com"

import rospy
from actionlib import SimpleActionServer

from geometry_msgs.msg import PoseStamped
from pandora_navigation_msgs.msg import DoExplorationAction, \
    DoExplorationFeedback, DoExplorationResult
import tf

class MockExplorer():

    def __init__(self, exploration_topic):

        self.robot_pose_ = PoseStamped()
        self.listener = tf.TransformListener()

        self.navigation_succedes = True
        self.reply = False
        self.preempted = 0
        
        self.entered_exploration = False

        self.do_exploration_as_ = SimpleActionServer(
            exploration_topic,
            DoExplorationAction,
            execute_cb = self.do_exploration_cb,
            auto_start = False)
        self.do_exploration_as_.start()

    def __del__(self):

        self.do_exploration_as_.__del__()

    def do_exploration_cb(self, goal):
        rospy.loginfo('do_exploration_cb')

        self.entered_exploration = True
        while not self.reply:
            rospy.sleep(0.2)
            (trans, rot) = self.listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
            self.robot_pose_.pose.position.x = trans[0]
            self.robot_pose_.pose.position.y = trans[1]
            feedback = DoExplorationFeedback()
            feedback.base_position.pose.position.x = \
                self.robot_pose_.pose.position.x
            feedback.base_position.pose.position.y = \
                self.robot_pose_.pose.position.y
            self.do_exploration_as_.publish_feedback(feedback)
            if self.do_exploration_as_.is_preempt_requested():
                self.preempted += 1
                rospy.loginfo("Preempted!")
                self.entered_exploration = False
                self.do_exploration_as_.set_preempted(DoExplorationResult())
                return None
        else:
            result = DoExplorationResult()
            self.reply = False
            self.preempted = 0
            self.entered_exploration = False
            if self.navigation_succedes:
                self.do_exploration_as_.set_succeded(result) 
            else:
                self.do_exploration_as_.set_aborted(result)
    
if __name__ == '__main__':

    rospy.sleep(0.5)
    rospy.init_node('MockExplorer', anonymous=True)
    explorer = MockExploration(exploration_topic = '/do_exploration')

