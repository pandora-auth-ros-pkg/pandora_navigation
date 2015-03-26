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

#include "pandora_exploration/navfn_service_frontier_path_generator.h"

namespace pandora_exploration {

NavfnServiceFrontierPathGenerator::NavfnServiceFrontierPathGenerator(const std::string& name,
              const std::string& frontier_representation, ros::Duration max_duration)
  : pnh_("~"),
    FrontierPathGenerator(name, frontier_representation),
    max_duration_(max_duration)
{
  // service name will be loaded here from parameter server
  std::string service_name;
  pnh_.param<std::string>(name_ + "/plan_service_name", service_name, "move_base/make_plan");
  path_client_ = nh_.serviceClient<nav_msgs::GetPlan>(service_name);
}

bool NavfnServiceFrontierPathGenerator::findPaths(const geometry_msgs::PoseStamped& start,
                                                            const FrontierListPtr& frontier_list)
{
  // find service
  if (!path_client_.waitForExistence()) {
    ROS_ERROR("[%s] Could not connect to make_plan service!", ros::this_node::getName().c_str());
    return false;
  }

  // track start
  ros::Time start_time = ros::Time::now();

  // calculate path for each frontier
  BOOST_FOREACH(Frontier& frontier, *frontier_list)
  {
    nav_msgs::GetPlan srv;
    srv.request.start = start;

    srv.request.goal.header = frontier.header;
    
    // check to what point we want to plan
    if (frontier_representation_ == "centroid") {
      srv.request.goal.pose.position = frontier.centroid;
    }
    else if (frontier_representation_ == "middle") {
      srv.request.goal.pose.position = frontier.middle;
    }
    else {
      srv.request.goal.pose.position = frontier.initial;
    }

    // send a valid pose
    srv.request.goal.pose.orientation.w = 1.0;

    // call service
    if (!path_client_.call(srv)) {
      ROS_ERROR("[%s] make_plan did not respond!", ros::this_node::getName().c_str());
      return false;
    }

    frontier.path = srv.response.plan;

    // if max time alloted return
    if (ros::Time::now() - start_time > max_duration_)
    {
      ROS_DEBUG("[%s] Time expired!", ros::this_node::getName().c_str());
      break;
    }
  }

  return true;
}

} // namespace pandora_exploration
