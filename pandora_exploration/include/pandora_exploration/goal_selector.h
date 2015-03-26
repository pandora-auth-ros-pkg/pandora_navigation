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

#ifndef PANDORA_EXPLORATION_GOAL_SELECTOR_H
#define PANDORA_EXPLORATION_GOAL_SELECTOR_H

#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <geometry_msgs/PoseStamped.h>

namespace pandora_exploration {

/**
  * @class GoalSelector
  * @brief Provides an interface for goal selectors used in pandora exploration.
  * Every goal selector implementation must adhere to this interface.
  *
  * This is the interface for the class that will wrap-up all the implementation
  * of the goal selection.
  */
class GoalSelector {
 public:
  
  /**
    * @brief Finds next exploration goal
    * @param goal We pass the goal that is found to this argument
    * @return True if a goal is found
    */
  virtual bool findNextGoal(geometry_msgs::PoseStamped* goal) = 0;

  /**
    * @brief Sets a selected goal to a vector
    * @param selected_goal A goal that has been selected by findNextGoal
    */
  virtual void setSelectedGoal(const geometry_msgs::PoseStamped& selected_goal)
  {
    selected_goals_.push_back(selected_goal);
  }

  /**
    * @brief Destructor for the goal selector.
    */
  virtual ~GoalSelector()
  {
  }

 protected:
  /**
    * @brief Explicit constructor for the goal selector.
    * @param name The name of the goal selector
    */
  explicit GoalSelector(const std::string& name) : name_(name)
  {
  }

 protected:
  std::string name_;

  // A vector that holds all the goals that have been selected
  std::vector<geometry_msgs::PoseStamped> selected_goals_;
};

typedef boost::shared_ptr<GoalSelector> GoalSelectorPtr;

}  // namespace pandora_exploration

#endif  // PANDORA_EXPLORATION_GOAL_SELECTOR_H
