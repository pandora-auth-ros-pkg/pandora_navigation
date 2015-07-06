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
import numpy
import math
import map_patch_params as params
import map_utils as utils

from obstacle import Obstacle
from numpy import *
from math import *
from nav_msgs.msg import OccupancyGrid
from pandora_data_fusion_msgs.msg import ObstacleInfo
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion


class MapPatcher():
    """
    The purpose of this class is to create a OccupancyGrid Map patch that will
    be published to a costmap layer in the Global Costmap and a costmap layer
    in the Local Costmap. There are two kinds of patches: Hard Obstacles and
    Soft Obstacles. Hard Obstacles define a region that is not traversable while
    Soft Obstacles define a region that is traversable. The main reason for the
    creation of the MapPatcher is the RoboCup Rescue 2015 Competition and the
    curtain and debris challenges where only a planar laser with 2D SLAM does
    not give us enough traversability information.
        This implementation should ideally be located inside the
    ros navigation costmap2d layer plugins, or some other costmap. It is instead
    implemented this way due to the lack of versatility of the ros navigation
    costmap implementation.
    """

    def __init__(self):
        """ Constructor of the MapPatcher class.
        """
        # An occupancy grid to hold the hard obstacles
        self.hard_patch = OccupancyGrid()
        self.map_patch = OccupancyGrid()
        # Subscriber to SLAM to get the map info
        self.sub_slam_ = rospy.Subscriber(params.slamMapTopic, OccupancyGrid,
                                          self.slamCB)

        # Subscriber to the obstacles posted by data_fusion
        self.sub_soft_obstacle_ = rospy.Subscriber(
            params.obstacleTopic, ObstacleInfo, self.obstacleCB)

        # Subscriber to obstacles posted in OGM(OccupancyGridMap) format
        self.sub_hard_obstacle_ = rospy.Subscriber(
            params.obstacleOGMTopic, OccupancyGrid, self.obstacleOGMCB)

        # Publisher to the HardLayer
        self.pub_ = rospy.Publisher(params.patchTopic, OccupancyGrid)

        self._obstacle_list = []

    def slamCB(self, slamMap):
        """
        @brief Callback of the slam topic.

        Uses the MapMetaDeta of the SLAM map to initialize the map_patch. Then
        if the hard_patch map is not empty we update the map_patch with the
        hard_patch. Finally, we look at the obstacle list which holds all the
        obstacles coming from data_fusion and we place them upon the map_patch
        doing the necessary transformations.
        """

        # Init soft_obstacle map, using slam MapMetaDeta and set every cell to NO_INFO
        utils.initMap(self.map_patch, slamMap)

        # If the hard layer map is empty do nothing
        # TODO (dimkirts) handle hard obstacles
        if not self.hard_patch.data:
            pass
        # Else
        else:
            #utils.mapResizer(self.map_patch, self.hard_patch)
            utils.updateWithOverwrite(self.map_patch, self.hard_patch)

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
                # Check if the obstacle is in the right map frame
                if obs.map_frame_id_ != "/" + str(slamMap.header.frame_id):
                    rospy.logerr(
                        "[MapPatcher]Expected map frame[%s], obstacle map frame[%s]",
                        slamMap.header.frame_id, obs.map_frame_id_)
                    return

                minX_ = 0.0
                maxX_ = obs.length_
                minY_ = 0.0
                maxY_ = obs.width_
                if obs.type_ == 1:
                    cost = params.freeCost
                elif obs.type_ == 2:
                    cost = params.lethalCost
                elif obs.type_ == -1:
                    rospy.logerr(
                        "[MapPatcher]Obstacle message type -1, not initialized right")
                else:
                    rospy.logerr("Should never reach here")

                patch_width = maxX_ - minX_
                patch_height = maxY_ - minY_

                # The initial patch center
                x0 = patch_width / 2.0
                y0 = patch_height / 2.0

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
                        temp_x = (i - x0) * math.cos(th) - (
                            j - y0) * math.sin(th) + dx
                        temp_y = (i - x0) * math.sin(th) + (
                            j - y0) * math.cos(th) + dy

                        temp_x = utils.metersToCells(
                            temp_x, self.map_patch.info.resolution)
                        temp_y = utils.metersToCells(
                            temp_y, self.map_patch.info.resolution)
                        self.map_patch.data[
                            temp_x + self.map_patch.info.width * temp_y
                        ] = cost

            # Set the timestamp and publish the map_patch
            self.map_patch.header.stamp = rospy.Time.now()

            self.pub_.publish(self.map_patch)

    def obstacleOGMCB(self, ogmMsg):
        """
        @brief Callback of the hard_obstacle OGM topic
        This is the callback to the OGM incoming from the hard_obstacle
        detection node. The incoming OGM must adhere to the SLAM map.
        """
        # If the hard layer map is empty create a NO_INFORMATION map with the
        # same MapMetaData as the incoming map
        if not self.hard_patch.data:
            utils.initMap(self.hard_patch, ogmMsg)

        # Check if incoming OGM is the same as the one we hold
        # What should be done in resizing?
        if not utils.mapMatchingChecker(self.hard_patch, ogmMsg):
            rospy.logerr("[MapPatcher]Incoming OGM is not valid")
            return

        # Update the hard_patch the class is holding with the incoming OGM using
        # an update method
        utils.updateWithOverwrite(self.hard_patch, ogmMsg)

    def obstacleCB(self, obstacleMsg):
        """ Callback to the data_fusion obstacle topic.
        """
        # Check type of obstacle and quaternion
        if ((obstacleMsg.type != params.softObstacleType) and  # noqa
            (obstacleMsg.type != params.hardObstacleType)):  # noqa
            rospy.logerr(
                "[MapPatcher]You send me either a barrel or an invalid type. Type: %d",
                obstacleMsg.type)
            return

        quat = obstacleMsg.obstaclePose.pose.orientation
        if not utils.isQuaternionValid(quat):
            rospy.logerr(
                "[MapPatcher]An invalid quaternion was passed, containing NaNs or Infs")
            return

        if utils.quaternionNotInstantiated(quat):
            rospy.logerr(
                "[MapPatcher]An invalid quaternion was passed, containing all zeros")
            return

        # Create an obstacle from the message of the callback
        obs = Obstacle()
        obs.createObstacle(obstacleMsg)

        # Search inside the list if we have an obstacle with the same id
        # If we do, we delete it and then append the new version of this obstacle
        # If we can't find a duplicate obstacle we append it
        for i in xrange(0, len(self._obstacle_list)):
            if self._obstacle_list[i].id_ == obs.id_:
                rospy.logwarn(
                    "[MapPatcher]Received new version of obstacle with id: %d",
                    self._obstacle_list[i].id_)
                rospy.loginfo("[MapPatcher]Replacing it in the obstacle list")
                del self._obstacle_list[i]
        self._obstacle_list.append(obs)


def main(args):
    ''' Initializes and cleanup ros node
    '''

    rospy.init_node('map_patcher')
    mapPatcher = MapPatcher()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.logwarn("Shutting down map_patcher!")


if __name__ == '__main__':
    main(sys.argv)
