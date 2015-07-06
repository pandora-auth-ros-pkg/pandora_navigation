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
import math
import map_patch_params as params

from math import *
from numpy import isfinite
from nav_msgs.msg import OccupancyGrid
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion

# # # # Map Patcher Utilities  # # # # #


def initMap(mapToSet, incomigMap):
    """
    @brief A function that initializes a map with NO_INFORMATION cells
    @param mapToSet The map to be initialized
    @param incomingMap The map which MapMetaData we use to initialize the
    MapToSet

    The mapToSet MapMetaData is set to be the same as the incoming map and
    it is initialized with NO_INFORMATION values.
    """

    mapToSet.header.frame_id = incomigMap.header.frame_id
    mapToSet.info = incomigMap.info

    # initialize the map with NO_INFORMATION cells
    temp_array = [params.unknownCost
                  ] * mapToSet.info.width * mapToSet.info.height
    mapToSet.data = temp_array
    mapToSet.info.origin.orientation.w = 1.0


def mapResizer(oldMap, newMap):
    """ A function that resizes the oldMap according to the new map
    MapMetaData, but keeps the old map data, doing the necessary
    transformations. The new map data are left intact.
    """
    old_size = len(oldMap.data)
    new_size = len(newMap.data)
    temp_old_map = oldMap.data

    oldMap.data = []  # clear old map
    oldMap.data = [params.unknownCost] * new_size  # resize the old map

    if (old_size == 0) or (new_size == 0):
        rospy.logerr("[Map Resizer] One of the two arrays is empty")
        return False

    if (oldMap.info.resolution == 0.0) or (newMap.info.resolution == 0.0):
        rospy.logerr("[Map Resizer] One of the two arrays has zero res")
        return False

    # Find x,y,yaw differences
    x_diff = newMap.info.origin.position.x - oldMap.info.origin.position.x
    y_diff = newMap.info.origin.position.y - oldMap.info.origin.position.y

    # Find old and new yaw to find yaw_diff
    old_quat = [
        oldMap.info.origin.orientation.x, oldMap.info.origin.orientation.y,
        oldMap.info.origin.orientation.z, oldMap.info.origin.orientation.w
    ]

    (old_roll, old_pitch, old_yaw) = euler_from_quaternion(old_quat)

    new_quat = [
        newMap.info.origin.orientation.x, newMap.info.origin.orientation.y,
        newMap.info.origin.orientation.z, newMap.info.origin.orientation.w
    ]

    (new_roll, new_pitch, new_yaw) = euler_from_quaternion(new_quat)

    yaw_diff = new_yaw - old_yaw
    res = newMap.info.resolution

    # Transform the old map to the new size
    for i in xrange(0, oldMap.info.width):
        for j in xrange(0, oldMap.info.height):
            # old x,y in meters
            x = i * oldMap.info.resolution
            y = j * oldMap.info.resolution

            # new x,y in meters
            xn = math.cos(yaw_diff) * x - math.sin(yaw_diff) * y - x_diff
            yn = math.sin(yaw_diff) * x + math.cos(yaw_diff) * y - y_diff
            print "x_diff: [" + str(x_diff) + "]"
            print "y_diff: [" + str(y_diff) + "]"
            # new x,y in cells
            xn_cell = int(round((xn / res), 0))
            yn_cell = int(round((yn / res), 0))

            coords = xn_cell + yn_cell * i - 1
            if (coords < 0) or (coords > new_size):
                rospy.logerr("[Map Resizer] Error in resizing xn_cell: [%d] \
                yn_cell: [%d] coords: [%d] new_size[%d]", xn_cell, yn_cell,
                             coords, new_size)
            else:
                temp = temp_old_map[i + j * oldMap.info.width]
                print "coords: " + str(coords)
                oldMap.data[coords] = temp
                # Dilation ??

                # Copy the MapMetaDeta of the new map
    oldMap.info = newMap.info
    # Copy the header (problems with the stamp?)
    oldMap.header = newMap.header
    return True


def mapMatchingChecker(currentMap, incomingMap):
    """
    @brief A function that checks if two maps have the same MapMetaData.
    Returns True if everything is OK.
    """
    # Check frame_id of incoming OGM
    if (incomingMap.header.frame_id != currentMap.header.frame_id):
        rospy.logerr("The incoming OGM has frame_id: [%s] the current map has \
        frame_id: [%s]", incomingMap.header.frame_id,
                     currentMap.header.frame_id)
        return False

    # Check resolution of incoming OGM
    if (incomingMap.info.resolution != currentMap.info.resolution):
        rospy.logerr(
            "The incoming OGM has resolution: [%f] the current map has \
        resolution: [%f]", incomingMap.info.resolution,
            currentMap.info.resolution)
        return False

    # Check width of incoming OGM
    if (incomingMap.info.width != currentMap.info.width):
        rospy.logerr("The incoming OGM has width: [%d] the current map has \
        width: [%d]", incomingMap.info.width, currentMap.info.width)
        return False

    # Check height of incoming OGM
    if (incomingMap.info.height != currentMap.info.height):
        rospy.logerr("The incoming OGM has height: [%d] the current map has \
        height: [%d]", incomingMap.info.height, currentMap.info.height)
        return False

    # Check if quaternion contains NaNs or Infs
    if not isQuaternionValid(incomingMap.info.origin.orientation):
        rospy.logerr(
            "An invalid quaternion was passed, containing NaNs or Infs")
        return False

    # Check if quaternion contains only zeros (not valid)
    if quaternionNotInstantiated(incomingMap.info.origin.orientation):
        rospy.logerr("An invalid quaternion was passed, containing all zeros")
        return False

    new_origin = incomingMap.info.origin
    old_origin = currentMap.info.origin
    # Check the origin of the OGM
    if ((new_origin.position.x != old_origin.position.x) or
        (new_origin.position.y != old_origin.position.y) or
        (new_origin.position.z != old_origin.position.z) or
        (new_origin.orientation.x != old_origin.orientation.x) or
        (new_origin.orientation.y != old_origin.orientation.y) or
        (new_origin.orientation.z != old_origin.orientation.z) or
        (new_origin.orientation.w != old_origin.orientation.w)):  # noqa
        rospy.logerr("The origins are different!")
        return False
    # Maybe check something about the time?
    # If everything is OK we return True
    return True


def updateWithOverwrite(mapToUpdate, incomingMap):
    """
    @brief A GridMap update method
    @param mapToUpdate The old map that will be updated
    @param incomingMap The new map that will overwrite the old map
    @return True If the update process is completed successfully, if not false

    This function updates the mapToUpdate OGM with the values
    of the incoming OGM. The NO_INFO values of the incomingMap DO NOT OVERWRITE
    the values of the old map.
    """
    if (mapToUpdate.info.width == 0 or mapToUpdate.info.height == 0 or
        incomingMap.info.width == 0 or incomingMap.info.height == 0):  # noqa
        rospy.logerr("[MapUtils] One of the maps has not been initialized")
        return False

    if not mapToUpdate.data:
        rospy.logerr("[MapUtils] mapToUpdate is empty")
        return False

    if not incomingMap.data:
        rospy.logerr("[MapUtils] incomingMap is empty")
        return False

    if mapToUpdate.info.width != incomingMap.info.width:
        rospy.logerr("[MapUtils] the two maps have different cell widths")
        return False

    if mapToUpdate.info.height != incomingMap.info.height:
        rospy.logerr("[MapUtils] the two maps have different cell heights")
        return False

    if len(mapToUpdate.data) != len(incomingMap.data):
        rospy.logerr("[MapUtils] Even though the two maps appear to have the \
        same width and height in the MapMetaData, their arrays have different \
        size")
        return False

    for i in xrange(0, incomingMap.info.width):
        for j in xrange(0, incomingMap.info.height):
            it = i + j * incomingMap.info.width
            if incomingMap.data[it] != params.unknownCost:
                mapToUpdate.data[it] = incomingMap.data[it]
    return True


def updateWithTrueOverwrite(mapToUpdate, incomingMap):
    """
    @brief A GridMap update method
    @param mapToUpdate The old map tha will be updated
    @param incomingMap The new map that will overwrite the old map
    @return True If the update process is completed successfully, if not false

    This function updates the mapToUpdate OGM with the values
    of the incoming OGM. The NO_INFO values of the incomingMap DO OVERWRITE
    the values of the old map.
    """
    if (mapToUpdate.info.width == 0 or mapToUpdate.info.height == 0 or
        incomingMap.info.width == 0 or incomingMap.info.height == 0):  # noqa
        rospy.logerr("[MapUtils] One of the maps has not been initialized")
        return False

    if not mapToUpdate.data:
        rospy.logerr("[MapUtils] mapToUpdate is empty")
        return False

    if not incomingMap.data:
        rospy.logerr("[MapUtils] incomingMap is empty")
        return False

    if mapToUpdate.info.width != incomingMap.info.width:
        rospy.logerr("[MapUtils] the two maps have different cell widths")
        return False

    if mapToUpdate.info.height != incomingMap.info.height:
        rospy.logerr("[MapUtils] the two maps have different cell heights")
        return False

    if len(mapToUpdate.data) != len(incomingMap.data):
        rospy.logerr("[MapUtils] Even though the two maps appear to have the \
        same width and height in the MapMetaData, their arrays have different \
        size")
        return False

    for i in xrange(0, incomingMap.info.width):
        for j in xrange(0, incomingMap.info.height):
            it = i + j * incomingMap.info.width
            mapToUpdate.data[it] = incomingMap.data[it]

    return True


def metersToCells(meters, resolution):
    """
    @brief Converts meters to cells given the cell resolution
    @param meters The meters we want to convert to cells
    @param resolution The resolution of the GridMap
    @return Returns the number of cells (int)
    """
    try:
        return int(round((meters / resolution), 3))
    except ZeroDivisionError:
        raise ZeroDivisionError("Division with zero")


def isQuaternionValid(quaternion):
    """
    @brief Check if a quaternion contains Infs or NaNs (not valid)
    @param quaternion The quaternion to check
    @return True If the quaternion is valid False if it contains Infs or NaNs

    A quaternion with all values zero is considered valid in this function
    """
    if (not isfinite(quaternion.x) or not isfinite(quaternion.y) or
        not isfinite(quaternion.z) or not isfinite(quaternion.w)):  # noqa
        return False
    return True


def quaternionNotInstantiated(quaternion):
    """ Checks if the given quaternion is not instantiated

    @param quaternion The quaternion to check
    @return True If the quaternion is (x,y,z,w) = (0,0,0,0), else False
    """
    if (quaternion.x == 0.0 and quaternion.y == 0.0 and quaternion.z == 0.0 and
        quaternion.w == 0.0):  # noqa
        return True
    return False
