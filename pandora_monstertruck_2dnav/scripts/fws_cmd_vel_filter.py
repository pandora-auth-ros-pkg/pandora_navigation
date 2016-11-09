#!/usr/bin/env python
# -*- coding: utf-8 -*-

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

__license__ = "BSD"
__copyright__ = "Copyright (c) 2016, P.A.N.D.O.R.A. Team. All rights reserved."
__author__ = "George Kouros"
__email__ = "gkourosg@yahoo.gr"

import rospy
from geometry_msgs.msg import Twist
from math import radians, atan

class FWSCmdVelFilter:

    def __init__(self):
        input_cmd_vel_topic = rospy.get_param("input_cmd_vel_topic", "/cmd_vel")
        output_cmd_vel_topic = rospy.get_param("output_cmd_vel_topic", "/robot0/cmd_vel")

        self.wheelbase = rospy.get_param("wheelbase", 0.32)
        self.min_turning_radius = rospy.get_param("min_turning_radius", 0.35)
        self.max_trans_vel_angle = rospy.get_param("max_trans_vel_angle", radians(23.0))

        self.cmd_vel_sub = rospy.Subscriber(input_cmd_vel_topic, Twist, self.cmd_vel_cb)
        self.cmd_vel_pub = rospy.Publisher(output_cmd_vel_topic, Twist, queue_size=10)
        rospy.spin()

    def cmd_vel_cb(self, data):
        try:
            if abs(data.linear.x / data.angular.z) < self.min_turning_radius:
                rospy.logwarn('[fws_cmd_vel_filter] Received Invalid Command: '
                              'R<Rmin. Command Not Relayed.')
                return
        except ZeroDivisionError:
            pass

        try:
            if abs(atan(data.linear.y / data.linear.x)) > self.max_trans_vel_angle:
                rospy.logwarn('[fws_cmd_vel_filter] Received Invalid Command: '
                              'β>βmax. Command Not Relayed.')
                return
        except ZeroDivisionError:
            if data.linear.x == 0 and data.linear.y != 0:
                rospy.logwarn('[fws_cmd_vel_filter] Received Invalid Command: '
                              'β>βmax. Command Not Relayed.')
                return

        self.cmd_vel_pub.publish(data)

if __name__ == '__main__':
    rospy.init_node('fws_cmd_vel_filter')
    filter = FWSCmdVelFilter()
