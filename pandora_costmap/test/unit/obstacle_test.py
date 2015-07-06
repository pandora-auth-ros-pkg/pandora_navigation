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
NAME = 'obstacle_test'

import sys
import unittest
import roslib

roslib.load_manifest('pandora_costmap')

from pandora_costmap import Obstacle
from pandora_data_fusion_msgs.msg import ObstacleInfo
from tf.transformations import euler_from_quaternion


class TestObstacle(unittest.TestCase):
    def setUp(self):
        self.obs = Obstacle()
        self.obsNorm = Obstacle()

    def tearDown(self):
        self.obs = None

    def test_createObstacle(self):
        # Case where obstacle is not initialized
        #obs = Obstacle()
        obsMsg = ObstacleInfo()
        self.obs.createObstacle(obsMsg)

        self.assertEqual(self.obs.length_, 0)
        self.assertEqual(self.obs.width_, 0)
        self.assertEqual(self.obs.id_, 0)
        self.assertEqual(self.obs.map_frame_id_, "")

        # Case where obstacle is initialized
        #obs_normal = Obstacle()

        obsMsgNorm = ObstacleInfo()

        # Message Creation
        obsMsgNorm.id = 0
        obsMsgNorm.obstacleFrameId = "soft_obstacle_0"
        obsMsgNorm.obstaclePose.header.frame_id = "/map"
        obsMsgNorm.obstaclePose.pose.position.x = 2.2
        obsMsgNorm.obstaclePose.pose.position.y = 2.2
        obsMsgNorm.obstaclePose.pose.position.z = 0.0

        obsMsgNorm.obstaclePose.pose.orientation.x = 0.0
        obsMsgNorm.obstaclePose.pose.orientation.y = 0.0
        obsMsgNorm.obstaclePose.pose.orientation.z = -0.06625
        obsMsgNorm.obstaclePose.pose.orientation.w = 0.9978
        obsMsgNorm.length = 2.0
        obsMsgNorm.width = 0.5
        obsMsgNorm.type = 1

        quat = [
            obsMsgNorm.obstaclePose.pose.orientation.x,
            obsMsgNorm.obstaclePose.pose.orientation.y,
            obsMsgNorm.obstaclePose.pose.orientation.z,
            obsMsgNorm.obstaclePose.pose.orientation.w
        ]

        (roll, pitch, yaw) = euler_from_quaternion(quat)

        # Obstacle creation
        self.obsNorm.createObstacle(obsMsgNorm)

        self.assertEqual(self.obsNorm.id_, 0)
        self.assertEqual(self.obsNorm.map_frame_id_, "/map")
        self.assertEqual(self.obsNorm.x_, 2.2)
        self.assertEqual(self.obsNorm.y_, 2.2)
        self.assertEqual(self.obsNorm.th_, yaw)
        self.assertEqual(self.obsNorm.length_, 2.0)
        self.assertEqual(self.obsNorm.width_, 0.5)
        self.assertEqual(self.obsNorm.type_, 1)
