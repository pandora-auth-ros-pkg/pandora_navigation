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

#include "pandora_exploration/frontier_goal_selector.h"

namespace pandora_exploration {

FrontierGoalSelector::FrontierGoalSelector(const std::string& name)
  : GoalSelector(name), tf_listener_(ros::Duration(10.0))
{
  ros::NodeHandle private_nh("~");
  explore_costmap_ros_.reset(new costmap_2d::Costmap2DROS(name_ + "_costmap", tf_listener_));
  frontier_marker_pub_ =
      private_nh.advertise<visualization_msgs::MarkerArray>(name_ + "_frontier_markers", 10);

  // path distance scale and straight scale
  double dist_scale;
  private_nh.param<double>(name_ + "/cost_functions/dist_scale", dist_scale, 1.0);

  // frontier size scale
  double size_scale;
  private_nh.param<double>(name_ + "/cost_functions/size_scale", size_scale, 1.0);

  // frontier alignment scale
  double alignment_scale;
  private_nh.param<double>(name_ + "/cost_functions/alignment_scale", alignment_scale, 1.0);

  // frontier alignment scale
  double visited_scale;
  private_nh.param<double>(name_ + "/cost_functions/visited_scale", alignment_scale, 1.0);

  // valid types: initial, middle, centroid
  private_nh.param<std::string>(name_ + "/frontier_representation", frontier_representation_,
                                "initial");

  // visualize paths to all frontiers
  private_nh.param<bool>(name_ + "/visualize_paths", visualize_paths_, false);

  // planner timeout duration;
  double duration;
  private_nh.param<double>(name_ + "/planner_timeout_duration", duration, 1.0);
  ros::Duration expiration(duration);

  // set-up map frontier search
  FrontierSearchPtr map_frontier_search(new MapFrontierSearch(
      boost::shared_ptr<costmap_2d::Costmap2D>(explore_costmap_ros_->getCostmap()),
      explore_costmap_ros_->getGlobalFrameID()));
  frontier_search_vec_.push_back(map_frontier_search);

  // set-up navfn path generator
  //frontier_path_generator_.reset( new NavfnFrontierPathGenerator(frontier_representation_,
  //explore_costmap_ros_) );
  frontier_path_generator_.reset(
      new NavfnServiceFrontierPathGenerator(name_, frontier_representation_, expiration));

  // set-up cost functions
  FrontierCostFunctionPtr size_cost_function(new SizeCostFunction(size_scale));
  frontier_cost_function_vec_.push_back(size_cost_function);
  FrontierCostFunctionPtr distance_cost_function(new DistanceCostFunction(dist_scale));
  frontier_cost_function_vec_.push_back(distance_cost_function);
  FrontierCostFunctionPtr alignment_cost_function(
      new AlignmentCostFunction(alignment_scale, current_robot_pose_));
  frontier_cost_function_vec_.push_back(alignment_cost_function);
  FrontierCostFunctionPtr visited_cost_function(
      new VisitedCostFunction(visited_scale, selected_goals_));
  frontier_cost_function_vec_.push_back(visited_cost_function);

  // set-up frontier list
  frontier_list_.reset(new FrontierList);
}

bool FrontierGoalSelector::findNextGoal(geometry_msgs::PoseStamped* goal)
{
  // clear previous frontiers
  frontier_list_->clear();

  // get robot pose
  tf::Stamped<tf::Pose> global_pose;
  explore_costmap_ros_->getRobotPose(global_pose);
  tf::poseStampedTFToMsg(global_pose, current_robot_pose_);

  // iterate over all frontier searches and find new frontiers
  BOOST_FOREACH(const FrontierSearchPtr & frontier_search, frontier_search_vec_)
  {
    FrontierList temp_list = frontier_search->searchFrom(current_robot_pose_.pose.position);

    // insert new frontiers to global list
    frontier_list_->insert(frontier_list_->end(), temp_list.begin(), temp_list.end());
  }

  // if no frontiers found return false
  if (frontier_list_->empty()) {
    ROS_ERROR("[%s] No frontiers found", ros::this_node::getName().c_str());
    return false;
  }

  // sort frontier list so closer frontier are first
  // sorting in respect with min_distance
  frontier_list_->sort();

  // find paths to all frontiers
  if (!frontier_path_generator_->findPaths(current_robot_pose_, frontier_list_)) {
    ROS_ERROR("[%s] Could not find paths to frontiers", ros::this_node::getName().c_str());
    return false;
  }

  // run all cost functions
  BOOST_FOREACH(const FrontierCostFunctionPtr & cost_function, frontier_cost_function_vec_)
  {
    cost_function->scoreFrontiers(frontier_list_);
  }

  // visualize new frontiers
  visualizeFrontiers();

  Frontier selected;

  // find the best frontier
  if (!findBestFrontier(&selected))
    return false;

  // correct final target orientation
  calculateFinalGoalOrientation(&selected);

  // keep track of selected frontier
  current_frontier_ = selected;

  // set as goal the last point in plan
  goal->header.frame_id = selected.header.frame_id;
  goal->header.stamp = selected.header.stamp;
  goal->pose = selected.path.poses.back().pose;

  return true;
}

bool FrontierGoalSelector::findBestFrontier(Frontier* selected)
{
  // search over all frontier to find max cost
  BOOST_FOREACH(Frontier frontier, *frontier_list_)
  {
    if (frontier.cost > selected->cost) {
      *selected = frontier;
    }
  }

  // did not found a frontier with positive cost
  if (selected->cost == 0.0)
    return false;

  return true;
}

void FrontierGoalSelector::calculateFinalGoalOrientation(Frontier* frontier)
{
  // path at least of 10 points, otherwise unstable orientations calculated
  if (frontier->path.poses.size() < 10)
    return;

  // find orientation of last 2 points
  geometry_msgs::Point final_point = frontier->path.poses.back().pose.position;
  geometry_msgs::Point semifinal_point =
      frontier->path.poses.at(frontier->path.poses.size() - 10).pose.position;
  double angle = atan2(final_point.y - semifinal_point.y, final_point.x - semifinal_point.x);

  // assign orientation to last point in path
  frontier->path.poses.back().pose.orientation = tf::createQuaternionMsgFromYaw(angle);
}

void FrontierGoalSelector::visualizeFrontiers()
{
  if (frontier_list_->empty())
    return;

  // no point to run, if no one is listening
  if (frontier_marker_pub_.getNumSubscribers() < 1)
    return;

  visualization_msgs::MarkerArray markers;

  int id = 0;

  // find also best frontier
  Frontier best;

  BOOST_FOREACH(const Frontier & frontier, *frontier_list_)
  {

    // visualize frontier as sphere
    visualization_msgs::Marker marker;
    marker.header.frame_id = explore_costmap_ros_->getGlobalFrameID();
    marker.header.stamp = ros::Time::now();
    marker.ns = "frontiers";
    marker.id = id++;
    if (frontier_representation_ == "centroid") {
      marker.pose.position.x = frontier.centroid.x;
      marker.pose.position.y = frontier.centroid.y;
    }
    else if (frontier_representation_ == "middle") {
      marker.pose.position.x = frontier.middle.x;
      marker.pose.position.y = frontier.middle.y;
    }
    else {
      marker.pose.position.x = frontier.initial.x;
      marker.pose.position.y = frontier.initial.y;
    }
    marker.pose.position.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 1.0;
    marker.color.a = 1.0;
    markers.markers.push_back(marker);

    // visualize frontier cost as marker text
    marker.ns = "total_cost";
    marker.pose.position.z = 0.2;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.text = boost::to_string(frontier.cost);
    markers.markers.push_back(marker);

    // visualize frontier size
    marker.ns = "size_cost";
    marker.text = boost::to_string(frontier.size);
    markers.markers.push_back(marker);

    // visualize path size
    marker.ns = "path_cost";
    marker.text = boost::to_string(frontier.path.poses.size());
    markers.markers.push_back(marker);

    // visualize paths to all frontiers as line strip
    if (visualize_paths_) {
      visualization_msgs::Marker paths_marker;
      paths_marker.ns = "paths";
      paths_marker.header.frame_id = explore_costmap_ros_->getGlobalFrameID();
      paths_marker.header.stamp = ros::Time::now();
      paths_marker.action = visualization_msgs::Marker::ADD;
      paths_marker.type = visualization_msgs::Marker::LINE_STRIP;
      paths_marker.id = id;
      paths_marker.scale.x = 0.01;
      paths_marker.color.g = 1.0;
      paths_marker.color.a = 1.0;
      paths_marker.pose.orientation.w = 1.0;

      BOOST_FOREACH(const geometry_msgs::PoseStamped & pose, frontier.path.poses)
      {
        paths_marker.points.push_back(pose.pose.position);
      }
      markers.markers.push_back(paths_marker);
    }

    // update best frontier
    if (frontier.cost > best.cost) {
      best = frontier;
    }
  }

  // add best frontier as blue sphere
  visualization_msgs::Marker marker;
  marker.header.frame_id = explore_costmap_ros_->getGlobalFrameID();
  marker.header.stamp = ros::Time::now();
  marker.ns = "best_frontier";
  marker.id = 0;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.15;
  marker.scale.y = 0.15;
  marker.scale.z = 0.15;
  marker.color.b = 1.0;
  marker.color.a = 1.0;
  if (frontier_representation_ == "centroid") {
    marker.pose.position.x = best.centroid.x;
    marker.pose.position.y = best.centroid.y;
  }
  else if (frontier_representation_ == "middle") {
    marker.pose.position.x = best.middle.x;
    marker.pose.position.y = best.middle.y;
  }
  else {
    marker.pose.position.x = best.initial.x;
    marker.pose.position.y = best.initial.y;
  }
  markers.markers.push_back(marker);

  // publish markers
  frontier_marker_pub_.publish(markers);
}

}  // namespace pandora_exploration
