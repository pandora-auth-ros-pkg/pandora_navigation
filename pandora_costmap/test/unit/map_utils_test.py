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
NAME = 'map_utils_test'

import sys
import unittest
import roslib

roslib.load_manifest('pandora_costmap')

from pandora_costmap import initMap, metersToCells, unknownCost
from pandora_costmap import isQuaternionValid, quaternionNotInstantiated
from pandora_costmap import updateWithTrueOverwrite, mapResizer
from pandora_costmap import updateWithOverwrite, mapMatchingChecker

from numpy import isfinite
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Quaternion


class TestMapUtils(unittest.TestCase):
    def test_metersToCells(self):
        meters = 10
        res = 0.01
        self.assertEqual(metersToCells(meters, res), 1000)
        self.assertRaises(ZeroDivisionError, metersToCells, meters, 0)
        self.assertEqual(metersToCells(0, 0.1), 0)

    def test_initMap(self):
        mapToSet = OccupancyGrid()
        incomingMap = OccupancyGrid()
        incomingMap.header.frame_id = "/world"
        incomingMap.info.origin.position.x = 5.0  # [m]
        incomingMap.info.origin.position.y = 4.0  # [m]
        incomingMap.info.origin.position.z = 3.0  # [m]
        incomingMap.info.origin.orientation.x = 0.0
        incomingMap.info.origin.orientation.y = -1.0
        incomingMap.info.origin.orientation.z = 0.0
        incomingMap.info.origin.orientation.w = 1.0
        incomingMap.info.resolution = 0.03  # [m/cell]
        incomingMap.info.width = 5  # [cells]
        incomingMap.info.height = 4  # [cells]
        # An array of 20 cells with values of 100
        incomingMap.data = [100] * 20
        # Call the function
        initMap(mapToSet, incomingMap)

        self.assertEqual(mapToSet.header.frame_id, "/world")
        self.assertEqual(mapToSet.info.origin.position.x, 5.0)  # [m]
        self.assertEqual(mapToSet.info.origin.position.y, 4.0)  # [m]
        self.assertEqual(mapToSet.info.origin.position.z, 3.0)  # [m]
        self.assertEqual(mapToSet.info.origin.orientation.x, 0.0)
        self.assertEqual(mapToSet.info.origin.orientation.y, -1.0)
        self.assertEqual(mapToSet.info.origin.orientation.z, 0.0)
        self.assertEqual(mapToSet.info.origin.orientation.w, 1.0)
        self.assertEqual(mapToSet.info.resolution, 0.03)  # [m/cell]
        self.assertEqual(mapToSet.info.width, 5)  # [cells]
        self.assertEqual(mapToSet.info.height, 4)  # [cells]
        self.assertEqual(mapToSet.data, [unknownCost] * 20)
        self.assertNotEqual(mapToSet.data, [100] * 20)

    def test_isQuaternionValid(self):
        quat = Quaternion()  # inits everything with zeros (0,0,0,0)

        self.assertEqual(quat.x, 0.0)
        self.assertEqual(quat.y, 0.0)
        self.assertEqual(quat.z, 0.0)
        self.assertEqual(quat.w, 0.0)
        self.assertTrue(isQuaternionValid(quat))

        nan = float('NaN')
        negative_inf = -float('Inf')
        positive_inf = float('Inf')

        quat.x = nan
        self.assertFalse(isQuaternionValid(quat))

        quat.x = negative_inf
        self.assertFalse(isQuaternionValid(quat))

        quat.x = positive_inf
        self.assertFalse(isQuaternionValid(quat))

        quat.x = nan
        quat.y = nan
        quat.z = positive_inf
        quat.w = negative_inf
        self.assertFalse(isQuaternionValid(quat))

        quat.x = nan
        quat.y = nan
        quat.z = nan
        quat.w = nan
        self.assertFalse(isQuaternionValid(quat))

    def test_quaternionNotInstantiated(self):
        quat = Quaternion()  # inits everything with zeros (0,0,0,0)

        self.assertEqual(quat.x, 0.0)
        self.assertEqual(quat.y, 0.0)
        self.assertEqual(quat.z, 0.0)
        self.assertEqual(quat.w, 0.0)
        self.assertTrue(quaternionNotInstantiated(quat))
        quat.x = 1.0
        self.assertFalse(quaternionNotInstantiated(quat))
        quat.x = float('NaN')
        self.assertFalse(quaternionNotInstantiated(quat))
        quat.x = float('Inf')
        self.assertFalse(quaternionNotInstantiated(quat))
        quat.x = -float('Inf')
        self.assertFalse(quaternionNotInstantiated(quat))

    def test_updateWithOverwrite(self):
        # Normal Cases
        old_OGM = OccupancyGrid()
        old_OGM.info.width = 5
        old_OGM.info.height = 3

        new_OGM = OccupancyGrid()
        new_OGM.info.width = 5
        new_OGM.info.height = 3
        # Case 1
        old_OGM.data = [0, 0, 0, 0, 100, 0, 0, 0, 0, 100, 0, 0, 0, 0, 100]
        new_OGM.data = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]

        self.assertTrue(updateWithOverwrite(old_OGM, new_OGM))
        # The old map is equal to the new because we don't have NO_INFOs
        self.assertEqual(old_OGM.data, new_OGM.data)

        # Case 2
        old_OGM.data = [0, 0, 0, 0, 100, 0, 0, 0, 0, 100, 0, 0, 0, 0, 100]
        new_OGM.data = [unknownCost] * 15

        self.assertTrue(updateWithOverwrite(old_OGM, new_OGM))
        # The old map didn't change because we only have NO_INFOs
        self.assertEqual(old_OGM.data, old_OGM.data)

        # Case 3
        old_OGM.data = [0, 0, 0, 0, 100, 0, 0, 0, 0, 100, 0, 0, 0, 0, 100]
        new_OGM.data = [unknownCost] * 15
        new_OGM.data[4] = 1
        new_OGM.data[9] = 2
        new_OGM.data[14] = 3

        self.assertTrue(updateWithOverwrite(old_OGM, new_OGM))
        # The old map change only at the cells without NO_INFO
        res = [0, 0, 0, 0, 1, 0, 0, 0, 0, 2, 0, 0, 0, 0, 3]
        self.assertEqual(old_OGM.data, res)

        # Extreme Cases
        old_OGMex = OccupancyGrid()
        old_OGMex.info.width = 5
        old_OGMex.info.height = 3

        new_OGMex = OccupancyGrid()
        new_OGMex.info.width = 5
        new_OGMex.info.height = 3

        # Empty list case
        old_OGMex.data = []
        self.assertFalse(updateWithOverwrite(old_OGMex, new_OGMex))

        # Both lists empty case
        old_OGMex.data = []
        new_OGMex.data = []
        self.assertFalse(updateWithOverwrite(old_OGMex, new_OGMex))

        # Lists with same meta data but different array size
        old_OGMex.data = [0, 0, 0]
        self.assertFalse(updateWithOverwrite(old_OGMex, new_OGMex))

        # Lists are ok but width and height are different
        old_OGMex.data = [0, 0, 0, 0, 100, 0, 0, 0, 0, 100, 0, 0, 0, 0, 100]
        new_OGMex.data = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]
        old_OGMex.info.width = 3
        self.assertFalse(updateWithOverwrite(old_OGMex, new_OGMex))

        # Width and height are the same but they are zero
        old_OGMex.info.width = 0
        old_OGMex.info.height = 0
        new_OGMex.info.height = 0
        new_OGMex.info.width = 0
        self.assertFalse(updateWithOverwrite(old_OGMex, new_OGMex))

        # Maps not initialized
        old_grid = OccupancyGrid()
        new_grid = OccupancyGrid()
        self.assertFalse(updateWithOverwrite(old_OGMex, new_OGMex))

    def test_updateWithTrueOverwrite(self):
        # Normal Cases
        old_OGM = OccupancyGrid()
        old_OGM.info.width = 5
        old_OGM.info.height = 3

        new_OGM = OccupancyGrid()
        new_OGM.info.width = 5
        new_OGM.info.height = 3
        # Case 1
        old_OGM.data = [0, 0, 0, 0, 100, 0, 0, 0, 0, 100, 0, 0, 0, 0, 100]
        new_OGM.data = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]

        self.assertTrue(updateWithTrueOverwrite(old_OGM, new_OGM))
        # The old map will always be equal to the new map
        self.assertEqual(old_OGM.data, new_OGM.data)

        # Case 2
        old_OGM.data = [0, 0, 0, 0, 100, 0, 0, 0, 0, 100, 0, 0, 0, 0, 100]
        new_OGM.data = [unknownCost] * 15

        self.assertTrue(updateWithTrueOverwrite(old_OGM, new_OGM))
        # The old map should change even though we have only NO_INFOs
        self.assertEqual(old_OGM.data, new_OGM.data)

        # Case 3
        old_OGM.data = [0, 0, 0, 0, 100, 0, 0, 0, 0, 100, 0, 0, 0, 0, 100]
        new_OGM.data = [unknownCost] * 15
        new_OGM.data[4] = 1
        new_OGM.data[9] = 2
        new_OGM.data[14] = 3

        self.assertTrue(updateWithTrueOverwrite(old_OGM, new_OGM))
        # The old map change at every cell
        res = [0, 0, 0, 0, 1, 0, 0, 0, 0, 2, 0, 0, 0, 0, 3]
        self.assertNotEqual(old_OGM.data, res)
        self.assertEqual(old_OGM.data, new_OGM.data)

        # Extreme Cases
        old_OGMex = OccupancyGrid()
        old_OGMex.info.width = 5
        old_OGMex.info.height = 3

        new_OGMex = OccupancyGrid()
        new_OGMex.info.width = 5
        new_OGMex.info.height = 3

        # Empty list case
        old_OGMex.data = []
        self.assertFalse(updateWithTrueOverwrite(old_OGMex, new_OGMex))

        # Both lists empty case
        old_OGMex.data = []
        new_OGMex.data = []
        self.assertFalse(updateWithTrueOverwrite(old_OGMex, new_OGMex))

        # Lists with same meta data but different array size
        old_OGMex.data = [0, 0, 0]
        self.assertFalse(updateWithTrueOverwrite(old_OGMex, new_OGMex))

        # Lists are ok but width and height are different
        old_OGMex.data = [0, 0, 0, 0, 100, 0, 0, 0, 0, 100, 0, 0, 0, 0, 100]
        new_OGMex.data = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]
        old_OGMex.info.width = 3
        self.assertFalse(updateWithTrueOverwrite(old_OGMex, new_OGMex))

        # Width and height are the same but they are zero
        old_OGMex.info.width = 0
        old_OGMex.info.height = 0
        new_OGMex.info.height = 0
        new_OGMex.info.width = 0
        self.assertFalse(updateWithTrueOverwrite(old_OGMex, new_OGMex))

        # Maps not initialized
        old_grid = OccupancyGrid()
        new_grid = OccupancyGrid()
        self.assertFalse(updateWithTrueOverwrite(old_OGMex, new_OGMex))

    @unittest.skip("Not yet implemented")
    def test_mapMatchingChecker(self):
        pass

    @unittest.skip("Not yet implemented")
    def test_mapResizer(self):
        pass
