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
import numpy
import math
import map_patch_params as params

from numpy import *
from math import *
from nav_msgs.msg import OccupancyGrid
from pandora_data_fusion_msgs.msg import ObstacleInfo
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion

class Obstacle():
  """ A class that implements an obstacle. It holds all the necessary data that define an obstacle """
  def __init__(self):
    self.id_ = -1
    self.frame_id_ = ""
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
    self.id_ = obstacleMsg.id
    self.frame_id = obstacleMsg.obstacleFrameId
    self.time_found_ = obstacleMsg.timeFound

    # Create the quaternion to pass it
    quat = [
      obstacleMsg.obstaclePose.pose.orientation.x,
      obstacleMsg.obstaclePose.pose.orientation.y,
      obstacleMsg.obstaclePose.pose.orientation.z,
      obstacleMsg.obstaclePose.pose.orientation.w
    ]

    (roll,pitch,yaw) = euler_from_quaternion(quat)

    self.th_ = yaw

    self.x_ = obstacleMsg.obstaclePose.pose.position.x
    self.y_ = obstacleMsg.obstaclePose.pose.position.y

    self.length_ = obstacleMsg.length
    self.width_ = obstacleMsg.width

    self.type_ = obstacleMsg.type

class MockMapPatcher():
  """ A class that implements a node that sends the appropriate data to the hard layer """
  # Constructor
  def __init__(self):

    self.map_patch = OccupancyGrid()
    # Subscriber to SLAM to get the map info
    self.sub_slam_ = rospy.Subscriber(params.slamMapTopic, OccupancyGrid, self.slamCB)

    # Subscriber to the obstacles posted by data_fusion
    self.sub_obstacle_ = rospy.Subscriber(params.obstacleTopic, ObstacleInfo, self.obstacleCB)
    # Publisher to the HardLayer
    self.pub_ = rospy.Publisher(params.patchTopic, OccupancyGrid)

    self._obstacle_list = []


  def metersToCels(self, meters, res):
    return int(round( (meters/res), 3))

  def isQuaternionValid(self, quaternion):
    if not isfinite(quaternion.x) or not isfinite(quaternion.y) or \
    not isfinite(quaternion.z) or not isfinite(quaternion.w):
      return False
    return True

  def quaternionNotInstantiated(self, quaternion):
    if quaternion.x == 0.0 and quaternion.y == 0.0 and quaternion.z == 0.0 and \
    quaternion.w == 0.0:
      return True
    return False

  def obstacleCB(self, obstacleMsg):
    """ Callback to the data_fusion obstacle topic. """
    # Check type of obstacle and quaternion
    if (obstacleMsg.type != params.softObstacleType) and (obstacleMsg.type != params.hardObstacleType):
      rospy.logerr("You send me either a barrel or an invalid type. Type: %d", obstacleMsg.type)
      return

    if not self.isQuaternionValid(obstacleMsg.obstaclePose.pose.orientation):
      rospy.logerr("An invalid quaternion was passed, containing NaNs or Infs")
      return

    if self.quaternionNotInstantiated(obstacleMsg.obstaclePose.pose.orientation):
      rospy.logerr("An invalid quaternion was passed, containing all zeros")
      return

    # Create an obstacle from the message of the callback
    obs = Obstacle()
    obs.createObstacle(obstacleMsg)

    # Search inside the list if we have an obstacle with the same id
    # If we do, we delete it and then append the new version of this obstacle
    # If we can't find a duplicate obstacle we append it
    for i in xrange(0, len(self._obstacle_list) ):
      if self._obstacle_list[i].id_ == obs.id_:
          rospy.logwarn("Received new version of obstacle with id: %d", self._obstacle_list[i].id_)
          rospy.loginfo("Replacing it in the obstacle list")
          del self._obstacle_list[i]
    self._obstacle_list.append(obs)

  def slamCB(self, slamMap):
    """ Callback from the slam topic. It copies all the map info and then adds the patch
    and publishes the new map to the HardLayer """

    # Save data from slam map

    self.map_patch.header.frame_id = slamMap.header.frame_id
    self.map_patch.info.resolution = slamMap.info.resolution
    self.map_patch.info.origin.position.x = slamMap.info.origin.position.x
    self.map_patch.info.origin.position.y = slamMap.info.origin.position.y
    self.map_patch.info.origin.position.z = slamMap.info.origin.position.z
    self.map_patch.info.width = slamMap.info.width
    self.map_patch.info.height = slamMap.info.height
    # debug print
    #print "res = " + str(self.map_patch.info.resolution)
    # Fill origin orientation
    quat = quaternion_from_euler(0, 0, 0) # (roll, pitch, yaw), in rads
    self.map_patch.info.origin.orientation.x = quat[0]
    self.map_patch.info.origin.orientation.y = quat[1]
    self.map_patch.info.origin.orientation.z = quat[2]
    self.map_patch.info.origin.orientation.w = quat[3]

    # initialize the map with NO_INFORMATION cells
    temp_array = [params.unknownCost] * self.map_patch.info.width * self.map_patch.info.height
    self.map_patch.data = temp_array

    # If obstacle list is empty then we just fill the map layer with no information
    # and then we post it.
    # If we have obstacles in the list we transform each object and publish the map

    # If list is empty
    if not self._obstacle_list:
      # Set the timestamp
      self.map_patch.header.stamp = rospy.Time.now()
      # Publish the map with NO_INFORMATION
      self.pub_.publish(self.map_patch)

    else:
      # If obstacle list is not empty
      # Set the patch, in meters to the bottom left corner, yaw in radians
      # 1.57 rad = 90 deg, 0.785 rad = 45 deg
      for obs in self._obstacle_list:
        minX_ = 0.0
        maxX_ = obs.length_
        minY_ = 0.0
        maxY_ = obs.width_
        if obs.type_ == 1:
            cost = params.freeCost
        elif obs.type_ == 2:
            cost = params.lethalCost
        elif obs.type_ == -1:
            rospy.logerr("Obstacle message type -1, not initialized right")
        else:
            rospy.logerr("Should never reach here")


        patch_width = maxX_ - minX_
        patch_height = maxY_ - minY_

        # The initial patch center
        x0 = patch_width/2.0
        y0 = patch_height/2.0

        # The new center is the position we get from the obstacle_msg
        # Position is in meters, th in radians, we rotate clockwise so the
        # angle th is negative

        th = obs.th_
        xn = obs.x_
        yn = obs.y_

        dx = xn
        dy = yn
        # print str(dx)+" "+str(dy)

        iterX = numpy.linspace(0.0, maxX_, maxX_ / 0.01)
        iterY = numpy.linspace(0.0, maxY_, maxY_ / 0.01)

        # Transformation (Rotation and Translation)
        for i in iterX:
            for j in iterY:
                temp_x = (i-x0)*math.cos(th) - (j-y0)*math.sin(th) + dx
                temp_y = (i-x0)*math.sin(th) + (j-y0)*math.cos(th) + dy

                temp_x = self.metersToCels(temp_x, self.map_patch.info.resolution)
                temp_y = self.metersToCels(temp_y, self.map_patch.info.resolution)
                self.map_patch.data[temp_x + self.map_patch.info.width * temp_y] = cost

    # Set the timestamp
    self.map_patch.header.stamp = rospy.Time.now()

    self.pub_.publish(self.map_patch)

def main(args):
  ''' Initializes and cleanup ros node '''

  rospy.init_node('mock_map_patcher')
  mockMapPatcher = MockMapPatcher()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    rospy.logwarn("Shutting down mock_soft_patcher!")

if __name__ == '__main__':
  main(sys.argv)
