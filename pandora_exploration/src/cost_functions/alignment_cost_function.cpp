/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014 - 2015, P.A.N.D.O.R.A. Team.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the P.A.N.D.O.R.A. Team nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Chris Zalidis <zalidis@gmail.com>,
          Dimitrios Kirtsios <dimkirts@gmail.com>
*********************************************************************/

#include "pandora_exploration/cost_functions/alignment_cost_function.h"

namespace pandora_exploration {

AlignmentCostFunction::AlignmentCostFunction(double scale,
                                             const geometry_msgs::PoseStamped& robot_pose)
  : FrontierCostFunction(scale), robot_pose_(robot_pose)
{
}

void AlignmentCostFunction::scoreFrontiers(const FrontierListPtr& frontier_list)
{
  // iterate over all frontiers
  BOOST_FOREACH(Frontier & frontier, *frontier_list)
  {
    // if frontier has already negative cost no point to run this cost function
    if (frontier.cost < 0) {
      continue;
    }

    // select a point in path to find angle to
    // if no valid plan to frontier, cost should already be negative
    // and this function will never run
    geometry_msgs::PoseStamped goal_pose;
    if (frontier.path.poses.size() < 10) {
      goal_pose = frontier.path.poses.back();
    } else {
      // this is done to calculate the true alignment to goal of the robot footprint
      goal_pose = frontier.path.poses[9];
    }

    // find angle from robot to goal
    double angle_to_goal = atan2(goal_pose.pose.position.y - robot_pose_.pose.position.y,
                                 goal_pose.pose.position.x - robot_pose_.pose.position.x);

    // find shortest angle, result from -pi to pi
    double final_angle =
        angles::shortest_angular_distance(angle_to_goal, tf::getYaw(robot_pose_.pose.orientation));

    // update frontier's cost
    frontier.cost += scale_ * cos(final_angle / 2.0);
  }
}

}  // namespace pandora_exploration
