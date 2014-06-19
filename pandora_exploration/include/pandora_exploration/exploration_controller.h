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

#ifndef PANDORA_EXPLORATION_EXPLORATION_CONTROLLER_H
#define PANDORA_EXPLORATION_EXPLORATION_CONTROLLER_H

#include <ros/ros.h>

#include <boost/foreach.hpp>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <pandora_navigation_msgs/DoExplorationAction.h>
#include <move_base_msgs/MoveBaseAction.h>

#include "pandora_exploration/frontier_goal_selector.h"

namespace pandora_exploration {

  typedef actionlib::SimpleActionServer<pandora_navigation_msgs::DoExplorationAction> DoExplorationServer;
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
  
  class ExplorationController
  {
   public:
    ExplorationController();

    void executeCb(const pandora_navigation_msgs::DoExplorationGoalConstPtr &goal);
    void feedbackMovingCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);
    void doneMovingCb(const actionlib::SimpleClientGoalState& state,
                              const move_base_msgs::MoveBaseResultConstPtr& result);


   private:
  
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    DoExplorationServer do_exploration_server_;
    MoveBaseClient move_base_client_;

    std::vector<GoalSelectorPtr> goal_selectors_; 
  };

} // namespace pandora_exploration

#endif // PANDORA_EXPLORATION_EXPLORATION_CONTROLLER_H
