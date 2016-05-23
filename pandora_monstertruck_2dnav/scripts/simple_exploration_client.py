#!/usr/bin/env python

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

from sys import argv

import rospy
import frontier_exploration.msg
from geometry_msgs.msg import Point32, PointStamped, PolygonStamped
import actionlib

class SimpleExplorationClient:
    """ A simple client class that submits a goal to the frontier_exploration node"""

    def __init__(self, args):
        if len(args) == 1:
            max_x = int(argv[1])
            max_y = int(argv[1])
        elif len(argv) == 2:
            max_x = int(argv[1])
            max_y = int(argv[2])
        else:
            max_x = 10
            max_y = 10

        self.points_x = [max_x, max_x, -max_x, -max_x, max_x+0.1]
        self.points_y = [max_y, -max_y, -max_y, max_y, max_y+0.1]

    def request_exploration(self):
        client = actionlib.SimpleActionClient(
            'explore_server',
            frontier_exploration.msg.ExploreTaskAction)
        client.wait_for_server()

        #  exploration_goal.explore_boundary.header.frame_id = 'world'
        polygonStamped = PolygonStamped()
        polygonStamped.header.frame_id = 'world'
        polygonStamped.header.stamp = rospy.Time.now()
        point = Point32()
        initialGoal = PointStamped()
        initialGoal.header.frame_id = 'world'
        initialGoal.point = Point32(x=0, y=0, z=0)

        for x,y in zip(self.points_x, self.points_y):
            polygonStamped.polygon.points.append(Point32(x=x, y=y, z=0))

        exploration_goal = frontier_exploration.msg.ExploreTaskGoal()
        exploration_goal.explore_boundary = polygonStamped
        exploration_goal.explore_center = initialGoal

        client.send_goal(exploration_goal)
        client.wait_for_result()


if __name__ == '__main__':
    rospy.init_node('exploration_boundary_setter')
    simpleExplorationClient = SimpleExplorationClient(argv[1:])
    simpleExplorationClient.request_exploration()
