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

#include "pandora_exploration/exploration_controller.h"

namespace pandora_exploration {

ExplorationController::ExplorationController() :
  private_nh_("~"),
  do_exploration_server_(nh_, "do_exploration", boost::bind(&ExplorationController::executeCb, this, _1), false),
  move_base_client_("move_base", true)
{
  frontier_search_.resert(
            new FrontierSearch(boost::shared_ptr<costmap_2d::Costmap2D>(explore_costmap_ros_->getCostmap())) );
//~   do_exploration_server_.registerPreemptCallback(boost::bind(&ExplorationController::preemptCb, this));
  do_exploration_server_.start();
}

void ExplorationController::executeCb(const pandora_navigation_msgs::DoExplorationGoalConstPtr &goal)
{
  //wait for move_base to set-up
  if (move_base_client_.waitForServer()) {
    do_exploration_server_.setAborted(DoExplorationResult(), "could not connect to move_base action");
  }

  while (ros::ok() && do_exploration_server_.isActive())
  {
  }

  
}

void ExplorationController::feedbackMovingCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
{

}

void ExplorationController::doneMovingCb(const actionlib::SimpleClientGoalState& state,
                                            const move_base_msgs::MoveBaseResultConstPtr& result)
{
  
}


} // namespace pandora_exploration
