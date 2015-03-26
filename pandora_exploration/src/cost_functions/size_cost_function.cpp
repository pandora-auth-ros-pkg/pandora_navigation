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
// for M_E usage
#include <cmath>
#include "pandora_exploration/cost_functions/size_cost_function.h"

namespace pandora_exploration {

SizeCostFunction::SizeCostFunction(double scale) : FrontierCostFunction(scale)
{
}

void SizeCostFunction::scoreFrontiers(const FrontierListPtr& frontier_list)
{
  int max_size = 0;
  // iterate over all frontiers and find max distance
  BOOST_FOREACH(const Frontier & frontier, *frontier_list)
  {
    if (frontier.size > max_size) {
      max_size = frontier.size;
    }
  }

  // iterate over all frontiers
  BOOST_FOREACH(Frontier & frontier, *frontier_list)
  {
    // if frontier has a already negative cost no point to run this cost function
    if (frontier.cost < 0) {
      continue;
    }
    // update frontier's cost
    frontier.cost +=
        scale_ *
        (1.0 - exp(-static_cast<double>(frontier.size) / static_cast<double>(max_size) * M_E));
        // Cost are normalized, frontier with the highest size scores 1.
  }
}

}  // namespace pandora_exploration
