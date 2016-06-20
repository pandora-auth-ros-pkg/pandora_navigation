#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2016, P.A.N.D.O.R.A. Team.
# All rights reserved.
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
# 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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

__license__ = 'BSD'
__copyright__ = 'Copyright (c) 2016, P.A.N.D.O.R.A. Team. All rights reserved.'
__author__ = 'George Kouros'
__email__ = 'gkourosg@yahoo.gr'

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import tf

class Odom2BaseTfPublisher:

    def __init__(self):
        self.scan_sub = rospy.Subscriber('/odom', Odometry, self.odom_cb)
        self.trajectory_pub = rospy.Publisher('robot_trajectory', Path, queue_size=1);
        self.trajectory = Path()
        self.trajectory.header.frame_id = 'odom'

    def odom_cb(self, msg):
        br = tf.TransformBroadcaster()
        br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, 0),
                         (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                          msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
                         rospy.Time.now(), 'base_footprint', 'odom')
        br.sendTransform((0, 0, 0.14), (0, 0, 0, 1), rospy.Time.now(),
                        'base_link', 'base_footprint')
        self.trajectory.poses.append(PoseStamped())
        self.trajectory.poses[-1].pose = msg.pose.pose
        self.trajectory.poses[-1].header.frame_id = 'base_link'
        self.trajectory_pub.publish(self.trajectory)


if __name__ == '__main__':
    rospy.init_node('odom_to_base_tf_publisher')
    o2b = Odom2BaseTfPublisher()
    rospy.spin()
