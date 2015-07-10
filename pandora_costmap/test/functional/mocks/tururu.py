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
#	notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#	copyright notice, this list of conditions and the following
#	disclaimer in the documentation and/or other materials provided
#	with the distribution.
#  * Neither the name of P.A.N.D.O.R.A. Team nor the names of its
#	contributors may be used to endorse or promote products derived
#	from this software without specific prior written permission.
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

import math
import numpy
import roslib
roslib.load_manifest(PKG)
import sys
import rospy
from nav_msgs.msg import OccupancyGrid
import mock_params as params
from pandora_costmap import map_utils as utils


class MockHardMap():
    """
    This class implements a node that posts a map from SLAM "enhanced"
    with hard obstacles. It is used for testing purposes.
    """

    def __init__(self):
        # Publisher of the hard obstacle map
        self.map_pub = rospy.Publisher(params.hardMapTopic, OccupancyGrid,
                                       queue_size=1)
        self.slam_pub = rospy.Publisher(params.statMapTopic, OccupancyGrid,
                                        queue_size=1)
        # Subscriber to the SLAM map
        self.slam_sub = rospy.Subscriber(params.slamMapTopic, OccupancyGrid,
                                         self.slamCB)
        self.hard_map = OccupancyGrid()

    def slamCB(self, slamMap):

        utils.initMap(self.hard_map, slamMap)
        maxX_ = 2.0
        maxY_ = 2.0
        minX_ = 0.0
        minY_ = 0.0
        obs_x = 0.0
        obs_y = 0.0

        patch_width = maxX_ - minX_
        patch_height = maxY_ - minY_

        # The initial patch center
        x0 = patch_width / 2.0
        y0 = patch_height / 2.0

        # The new center is the position we get from the obstacle_msg
        # Position is in meters, th in radians, we rotate clockwise so the
        # angle th is negative
        # PLUS the position of the slam map
        th = 0.785
        xn = obs_x - slamMap.info.origin.position.x
        yn = obs_y - slamMap.info.origin.position.y

        dx = xn
        dy = yn
        # print str(dx)+" "+str(dy)

        iterX = numpy.linspace(0.0, maxX_, maxX_ / 0.1)
        iterY = numpy.linspace(0.0, maxY_, maxY_ / 0.1)

        # Transformation (Rotation and Translation)
        for i in iterX:
            for j in iterY:
                temp_x = (i - x0) * math.cos(th) - (j - y0) * math.sin(th) + dx
                temp_y = (i - x0) * math.sin(th) + (j - y0) * math.cos(th) + dy

                temp_x = utils.metersToCells(
                    temp_x, self.hard_map.info.resolution)
                temp_y = utils.metersToCells(
                    temp_y, self.hard_map.info.resolution)

                it = temp_x + self.hard_map.info.width * temp_y

                if it < 0 or it >= len(self.hard_map.data):
                    rospy.logerr(
                        "[MapPatcher]Index out of bounds dropping \
                        cell: [%d]!", it)
                else:
                    self.hard_map.data[it] = 99

        # Set the timestamp and publish the hard_map
        self.hard_map.header.stamp = rospy.Time.now()
        self.slam_pub.publish(slamMap)
        self.map_pub.publish(self.hard_map)


def main(args):
    ''' Initializes and cleanup ros node
    '''

    rospy.init_node('tururu')
    mockHardMap = MockHardMap()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.logwarn("Shutting down map_patcher!")


if __name__ == '__main__':
    main(sys.argv)
