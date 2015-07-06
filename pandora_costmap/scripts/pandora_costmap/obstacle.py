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

import rospy

from pandora_data_fusion_msgs.msg import ObstacleInfo
from tf.transformations import euler_from_quaternion


class Obstacle():
    """ A class that implements an obstacle. It holds all the necessary data that
    define an obstacle. An obstacle comes from data_fusion and defines a region
    where a clearing or marking should be done on a costmap layer.
    """

    def __init__(self):
        # We store only the data we use from the message
        self.id_ = -1
        # This is the frame id of the pose that defines the obstacle patch
        # Not the frame of the obstacle itself.
        self.map_frame_id_ = ""  # It must be /map
        self.time_found_ = rospy.Time()

        # in radians
        self.th_ = 0.0

        # in meters
        self.x_ = 0.0
        self.y_ = 0.0

        # in meters
        self.length_ = 0.0
        self.width_ = 0.0

        # if type equals 1 we have soft obstacle, if equals 2 we have hard obstacle
        self.type_ = -1

    def createObstacle(self, obstacleMsg):
        """ Creates an obstacle object using data from an obstacleInfo msg that is
        generally received from data_fusion.
        """
        self.id_ = obstacleMsg.id
        self.map_frame_id_ = obstacleMsg.obstaclePose.header.frame_id
        self.time_found_ = obstacleMsg.timeFound

        # Create the quaternion to pass it
        quat = [
            obstacleMsg.obstaclePose.pose.orientation.x,
            obstacleMsg.obstaclePose.pose.orientation.y,
            obstacleMsg.obstaclePose.pose.orientation.z,
            obstacleMsg.obstaclePose.pose.orientation.w
        ]

        (roll, pitch, yaw) = euler_from_quaternion(quat)

        self.th_ = yaw

        self.x_ = obstacleMsg.obstaclePose.pose.position.x
        self.y_ = obstacleMsg.obstaclePose.pose.position.y

        self.length_ = obstacleMsg.length
        self.width_ = obstacleMsg.width

        self.type_ = obstacleMsg.type
