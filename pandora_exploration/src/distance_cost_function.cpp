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

#include "pandora_exploration/distance_cost_function.h"

namespace pandora_exploration {

DistanceCostFunction::DistanceCostFunction(double scale)
  : FrontierCostFunction(scale)
{
}

void DistanceCostFunction::scoreFrontiers(const FrontierListPtr& frontier_list)
{
  //iterate over all frontiers 
  BOOST_FOREACH(Frontier & frontier, *frontier_list)
  {
    //if frontier has a already negative cost no point to run this cost function
    if (frontier.cost < 0) {
      continue;
    }
    
    //find path distance based on path
    //frontier.path

    //if no path found penalize frontier
    if (frontier.path.poses.empty()) {
      frontier.cost = -1.0;
      continue;
    }

    //size of path shows "straightness" and lenght of the path, this has to be reviewed
    double path_dist = static_cast<double>(frontier.path.poses.size());
    //update frontier's cost
    frontier.cost /= scale_ * path_dist; //consider normalizing over all frontiers
  }
}

} // namespace pandora_exploration
