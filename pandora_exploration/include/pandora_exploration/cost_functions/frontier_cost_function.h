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
* Authors: Chris Zalidis <zalidis@gmail.com>,
           Dimitrios Kirtsios <dimkirts@gmail.com>
*********************************************************************/

#ifndef PANDORA_EXPLORATION_COST_FUNCTIONS_FRONTIER_COST_FUNCTION_H
#define PANDORA_EXPLORATION_COST_FUNCTIONS_FRONTIER_COST_FUNCTION_H

#include "pandora_exploration/frontier.h"

namespace pandora_exploration {

  /**
    * @class FrontierCostFunction
    * @brief Provides an interface for the cost functions of the frontier goal selector. Every 
    * frontier cost function implementation must adhere to this interface. 
    */
  class FrontierCostFunction
  {
   public:

    /**
      * @brief Takes a list of frontiers and adds a cost to each one of them
      * @param frontier_list The list holding the frontiers to be evaluated
      */
    virtual void scoreFrontiers(const FrontierListPtr& frontier_list) = 0;

    /**
      * @brief Destructor for the FrontierCostFunction
      */
    virtual ~FrontierCostFunction() {}

   protected:

    /**
      * @brief Explicit constructor for FrontierCostFunction
      * @param scale The weight we set on this cost function
      */
    explicit FrontierCostFunction(double scale = 1.0) : scale_(scale) {}

   protected:

    double scale_;
  };

  typedef boost::shared_ptr<FrontierCostFunction> FrontierCostFunctionPtr;

}   // namespace pandora_exploration

#endif  // PANDORA_EXPLORATION_COST_FUNCTIONS_FRONTIER_COST_FUNCTION_H
