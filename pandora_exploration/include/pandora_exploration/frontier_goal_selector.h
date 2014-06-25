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

#ifndef PANDORA_EXPLORATION_FRONTIER_GOAL_SELECTOR_H
#define PANDORA_EXPLORATION_FRONTIER_GOAL_SELECTOR_H

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include "pandora_exploration/goal_selector.h"
#include "pandora_exploration/frontier.h"
#include "pandora_exploration/map_frontier_search.h"
#include "pandora_exploration/navfn_frontier_path_generator.h"
#include "pandora_exploration/navfn_service_frontier_path_generator.h"
#include "pandora_exploration/distance_cost_function.h"
#include "pandora_exploration/size_cost_function.h"

namespace pandora_exploration {

  class FrontierGoalSelector : public GoalSelector
  {
   public:

    FrontierGoalSelector();

    virtual bool findNextGoal(geometry_msgs::PoseStamped* goal);

    ~FrontierGoalSelector() {}

   private:

    bool findBestFrontier(Frontier* selected);
    void visualizeFrontiers();

   private:

    ros::Publisher frontier_marker_pub_;
    boost::shared_ptr<costmap_2d::Costmap2DROS> explore_costmap_ros_;
    FrontierListPtr frontier_list_;

    tf::TransformListener tf_listener_;
    
    std::vector<FrontierSearchPtr> frontier_search_vec_;
    std::vector<FrontierCostFunctionPtr> frontier_cost_function_vec_;
    FrontierPathGeneratorPtr frontier_path_generator_;

    std::string frontier_representation_;
    bool visualize_paths_;

    Frontier current_frontier_;

  };


} // namespace pandora_exploration

#endif // PANDORA_EXPLORATION_FRONTIER_GOAL_SELECTOR_H
