#! /usr/bin/env python

# Software License Agreement
__version__ = "0.0.1"
__status__ = "Production"
__license__ = "BSD"
__copyright__ = "Copyright (c) 2015, P.A.N.D.O.R.A. Team. All rights reserved."
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
__author__ = "Dimitrios Kirtsios"
__maintainer__ = "Dimitrios Kirtsios"
__email__ = "dimkirts@gmail.com"

PKG = 'pandora_costmap'

import roslib
roslib.load_manifest(PKG)
import sys
import rospy
from pandora_data_fusion_msgs.msg import ObstacleInfo
import mock_params as params


def talker():
    pub = rospy.Publisher(params.obstacleTopic, ObstacleInfo, queue_size=1)
    rospy.init_node('mock_data_fusion', anonymous=True)
    rate = rospy.Rate(1)  # 1Hz
    while not rospy.is_shutdown():
        # Do message stuff
        obs = ObstacleInfo()
        obs.id = 1
        obs.obstaclePose.header.frame_id = "/map"
        obs.timeFound = rospy.get_rostime()
        obs.obstaclePose.pose.position.x = 0.943540
        obs.obstaclePose.pose.position.y = -0.10643
        obs.obstaclePose.pose.position.z = 0
        obs.obstaclePose.pose.orientation.x = 0
        obs.obstaclePose.pose.orientation.y = 0
        obs.obstaclePose.pose.orientation.z = -0.6003
        obs.obstaclePose.pose.orientation.w = 0.7997
        obs.length = 10.0  # 0.9013425
        obs.width = 10.0  # 0.5
        obs.type = 2
        # publish message
        pub.publish(obs)

        # obs = ObstacleInfo()
        # obs.id = 2
        # obs.obstaclePose.header.frame_id = "map"
        # obs.timeFound = rospy.get_rostime()
        # obs.obstaclePose.pose.position.x = 9
        # obs.obstaclePose.pose.position.y = 9
        # obs.obstaclePose.pose.position.z = 0
        # obs.obstaclePose.pose.orientation.x = 0
        # obs.obstaclePose.pose.orientation.y = 0
        # obs.obstaclePose.pose.orientation.z = 0
        # obs.obstaclePose.pose.orientation.w = 1
        # obs.length = 1
        # obs.width = 1
        # obs.type = 2
        # publish message
        pub.publish(obs)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
