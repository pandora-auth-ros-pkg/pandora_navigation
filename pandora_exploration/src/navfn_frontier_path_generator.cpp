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

#include "pandora_exploration/navfn_frontier_path_generator.h"

namespace pandora_exploration {

NavfnFrontierPathGenerator::NavfnFrontierPathGenerator(
    const std::string& name, const std::string& frontier_representation,
    const boost::shared_ptr<costmap_2d::Costmap2DROS>& costmap_ros)
  : pnh_("~"),
    costmap_ros_(costmap_ros),
    FrontierPathGenerator(name, frontier_representation),
    planner_loader_("nav_core", "nav_core::BaseGlobalPlanner")
{
  // get planner's name
  std::string planner_name;
  pnh_.param<std::string>(name_ + "/global_planner", planner_name, "navfn/NavfnROS");

  // check if planner plugin exists
  if (!planner_loader_.isClassAvailable(planner_name)) {
    ROS_FATAL("[%s] Could not find planner plugin %s", ros::this_node::getName().c_str(),
              planner_name.c_str());
    ROS_BREAK();
  }

  // create planner, exception is throwed if plugin is not available
  try
  {
    planner_ = planner_loader_.createInstance(planner_name);
    planner_->initialize(planner_loader_.getName(planner_name), costmap_ros_.get());
  }
  catch (const pluginlib::PluginlibException& ex)
  {
    ROS_FATAL("[%s] %s", ros::this_node::getName().c_str(), ex.what());
    ROS_BREAK();
  }
}

bool NavfnFrontierPathGenerator::findPaths(const geometry_msgs::PoseStamped& start,
                                           const FrontierListPtr& frontier_list)
{
  // calculate path for each frontier
  BOOST_FOREACH(Frontier & frontier, *frontier_list)
  {
    // Here we will store the plan to 
    nav_msgs::Path plan;
    geometry_msgs::PoseStamped goal;

    goal.header = frontier.header;

    // check to what point we want to plan
    if (frontier_representation_ == "centroid") {
      goal.pose.position = frontier.centroid;
    }
    else if (frontier_representation_ == "middle") {
      goal.pose.position = frontier.middle;
    }
    else {
      goal.pose.position = frontier.initial;
    }
    goal.pose.orientation.w = 1.0;

    // start pose, goal pose, plan filled by the planner
    // TODO isws edw kati den paei kala, an xrhsimopoiithei autos o generator
    // to make plan epistrefei true an vrhke ena valid plan alliws false.
    planner_->makePlan(start, goal, plan.poses);

    plan.header = frontier.header;
    // store the calculated plan to frontier.path variable
    frontier.path = plan;
  }

  return true;
}

}  // namespace pandora_exploration
