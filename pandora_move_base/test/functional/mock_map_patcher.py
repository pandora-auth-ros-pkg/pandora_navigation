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

PKG = 'pandora_move_base'

import roslib
roslib.load_manifest(PKG)
import sys
import rospy

import mock_map_patcher_params as params

from nav_msgs.msg import OccupancyGrid
from tf.transformations import quaternion_from_euler


class MockMapPatcher():
  """ A class that implements a node that sends the appropriate data to the hard layer """
  # Constructor
  def __init__(self):

    self.mock_map = OccupancyGrid()
    # Subscriber to SLAM to get the map info
    self.sub_slam_ = rospy.Subscriber(params.slamMapTopic, OccupancyGrid, self.slamCb)

    # Subscriber to Vision to get the bounding box of the patch
    #self.sub_vision_ = rospy.Subscriber(params.visionTopic, OccupancyGrid, self.slamCb)
    #self.sub_vision_ = rospy.Subscriber(params.visionTopic, self.visionCb)
    # Publisher to the HardLayer
    self.pub_ = rospy.Publisher(params.hardLayerTopic, OccupancyGrid)
    

  def metersToCels(self, meters, res):
    return round( (meters/res), 3)


  def slamCb(self, slamMap):
    """ Callback from the slam topic. It copies all the map info and then adds the patch
    and publishes the new map to the HardLayer """
    # Copy data from slam map
    

    self.mock_map.header.frame_id = slamMap.header.frame_id
    self.mock_map.info.resolution = slamMap.info.resolution
    self.mock_map.info.origin.position.x = slamMap.info.origin.position.x
    self.mock_map.info.origin.position.y = slamMap.info.origin.position.y
    self.mock_map.info.origin.position.z = slamMap.info.origin.position.z
    self.mock_map.info.width = slamMap.info.width
    self.mock_map.info.height = slamMap.info.height
    
    # Fill origin orientation
    quat = quaternion_from_euler(0, 0, 0) # (roll, pitch, yaw), in rads
    self.mock_map.info.origin.orientation.x = quat[0]
    self.mock_map.info.origin.orientation.y = quat[1]
    self.mock_map.info.origin.orientation.z = quat[2]
    self.mock_map.info.origin.orientation.w = quat[3]

    # initialize the map with NO_INFORMATION cells
    temp_array = [51] * self.mock_map.info.width * self.mock_map.info.height
    self.mock_map.data = temp_array
    
    # Set the patch
    minX_ = 7
    maxX_ = 8
    minY_ = 7
    maxY_ = 8
    cost = 90

    minX_ = self.metersToCels(minX_, self.mock_map.info.resolution)
    maxX_ = self.metersToCels(maxX_, self.mock_map.info.resolution)
    minY_ = self.metersToCels(minY_, self.mock_map.info.resolution)
    maxY_ = self.metersToCels(maxY_, self.mock_map.info.resolution)

    for i in range(0, self.mock_map.info.width):
      if i >= minX_ and i <= maxX_:
        for j in range(0, self.mock_map.info.height):
          if j >= minY_ and j <= maxY_:
            self.mock_map.data[i + self.mock_map.info.width * j] = cost

    # Set the timestamp
    self.mock_map.header.stamp = rospy.Time.now()

    self.pub_.publish(self.mock_map)

def main(args):
  ''' Initializes and cleanup ros node '''
  
  rospy.init_node('mock_map_patcher')
  mockMapPatcher = MockMapPatcher()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down mock_soft_patcher!"

if __name__ == '__main__':
  main(sys.argv)