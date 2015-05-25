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

#ifndef PANDORA_EXPLORATION_FRONTIER_GOAL_SELECTOR_H
#define PANDORA_EXPLORATION_FRONTIER_GOAL_SELECTOR_H

#include <string>
#include <vector>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include "pandora_exploration/goal_selector.h"
#include "pandora_exploration/frontier.h"
#include "pandora_exploration/map_frontier_search.h"
#include "pandora_exploration/navfn_frontier_path_generator.h"
#include "pandora_exploration/navfn_service_frontier_path_generator.h"

#include "pandora_exploration/cost_functions/distance_cost_function.h"
#include "pandora_exploration/cost_functions/size_cost_function.h"
#include "pandora_exploration/cost_functions/alignment_cost_function.h"
#include "pandora_exploration/cost_functions/visited_cost_function.h"

namespace pandora_exploration {

/**
  * @class FrontierGoalSelector
  * @brief A class implementing a goal selector using the GoalSelector interface
  *
  * This goal selector is using the idea of frontiers to choose it's next 
  * exploration target. The possible goals are frontiers of the
  * explored map.
  */
class FrontierGoalSelector : public GoalSelector {
 public:
  
  /**
    * @brief Explicit constructor for the FrontierGoalSelector class
    * @param name The name of the goal selector
    * 
    * Inside the constructor the params for the scales of the cost 
    * functions are loaded. Also the setup of exploration costmap,
    * map frontier searchers, frontier path generators, frontier 
    * cost functions, frontier list takes place.
    */
  explicit FrontierGoalSelector(const std::string& name);

  /**
    * @brief Finds next exploration goal
    * @param goal We pass the goal that is found to this argument
    * @return True if a goal is found
    */
  virtual bool findNextGoal(geometry_msgs::PoseStamped* goal);

  /**
    * @brief Destructor for the frontier goal selector.
    */
  ~FrontierGoalSelector()
  {
  }

 private:
  
  /**
    * @brief Finds the best frontier from the frontier list and passes it
    * to the frontier ptr we pass as argument
    * @params selected Frontier ptr to passed with the best frontier found
    * @return True if a frontier with positive cost is found
    *
    * The frontier with the max cost is choosed
    */
  bool findBestFrontier(Frontier* selected);
  
  /**
    * @brief Calculates the final orientation of the goal
    * @param frontier The goal frontier
    * 
    * Calculates the final goal orientation using the two last points 
    * of the frontier path and assigns this orientation to the last point
    * of the frontier path
    */
  void calculateFinalGoalOrientation(Frontier* frontier);
  
  /**
    * @brief Implements the visualization procedure in rviz
    * 
    * Each frontier is represented as red SHERE
    * The path to each frontier is represented as green LINE_STRIP
    * Best frontier is represented as blue SPHERE
    * 
    */
  void visualizeFrontiers();

 private:
  // Marker publisher used for visualization purposes
  ros::Publisher frontier_marker_pub_;
  
  // A shared ptr to the Costmap2DROS 
  boost::shared_ptr<costmap_2d::Costmap2DROS> explore_costmap_ros_;
  
  // A shared ptr to an std::list of frontier objects 
  FrontierListPtr frontier_list_;

  tf::TransformListener tf_listener_;

  geometry_msgs::PoseStamped current_robot_pose_;

  // A vector that holds pointers to all frontier searcher implementations
  std::vector<FrontierSearchPtr> frontier_search_vec_;

  // A vector that holds pointers to all frontier cost function implementations
  std::vector<FrontierCostFunctionPtr> frontier_cost_function_vec_;

  // A shared ptr to a frontier path generator
  FrontierPathGeneratorPtr frontier_path_generator_;

  std::string frontier_representation_;
  bool visualize_paths_;

  Frontier current_frontier_;
};

}  // namespace pandora_exploration

#endif  // PANDORA_EXPLORATION_FRONTIER_GOAL_SELECTOR_H
