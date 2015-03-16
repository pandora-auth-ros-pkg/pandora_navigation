/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, P.A.N.D.O.R.A. Team.
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
* Author: Chris Zalidis <zalidis@gmail.com>
*********************************************************************/

#include "pandora_exploration/cost_functions/visited_cost_function.h"

namespace pandora_exploration {

VisitedCostFunction::VisitedCostFunction(
    double scale, const std::vector<geometry_msgs::PoseStamped>& selected_goals)
  : FrontierCostFunction(scale), selected_goals_(selected_goals)
{
}

void VisitedCostFunction::scoreFrontiers(const FrontierListPtr& frontier_list)
{
  // iterate over all frontiers
  BOOST_FOREACH(Frontier & frontier, *frontier_list)
  {
    // if frontier has a already negative cost no point to run this cost function
    if (frontier.cost < 0) {
      continue;
    }

    // how many times we have send a similar goal
    int times_seen = 0;
    double time = 0;

    // iterate over all previous goals
    BOOST_FOREACH(const geometry_msgs::PoseStamped & selected_goal, selected_goals_)
    {
      // find distance between selected_goal and frontier
      // using initial point, maybe get from param later
      double dx = selected_goal.pose.position.x - frontier.initial.x;
      double dy = selected_goal.pose.position.y - frontier.initial.y;

      if (::hypot(dx, dy) < 0.2) {
        time += (frontier.header.stamp - selected_goal.header.stamp).toSec();
        times_seen++;
      }
    }

    //    std::cout << (1.0 - pow((1.0/freq), 1.0/5.0)) << std::endl;
    //    std::cout << exp(-static_cast<double>(times_seen)) << std::endl;

    // update cost
    //    frontier.cost += scale_ * (1.0 - pow((1.0/freq), 1.0/5.0));
    frontier.cost += scale_ * exp(-static_cast<double>(times_seen));
  }
}

}  // namespace pandora_exploration
