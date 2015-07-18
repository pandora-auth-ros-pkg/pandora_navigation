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

#include <boost/algorithm/string.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "pandora_costmap/FilterConfig.h"
#include "pandora_costmap/point_cloud_cropper.h"

namespace pandora_costmap
{

  PointCloudCropper::PointCloudCropper() : nh_(), pnh_("~")
  {
    nodeName_ = boost::to_upper_copy<std::string>(pnh_.getNamespace());
    pnh_.param("reference_frame", reference_frame_, std::string("/base_footprint"));

    if (!pnh_.getParam("in_topic", in_topic_))
    {
      ROS_FATAL("[%s] Could not find in topic!", nodeName_.c_str());
      ROS_BREAK();
    }

    if (!pnh_.getParam("out_topic", out_topic_))
    {
      ROS_FATAL("[%s] Could not find out topic!", nodeName_.c_str());
      ROS_BREAK();
    }

    cloudInSubscriber_ =
        nh_.subscribe(in_topic_, 1, &PointCloudCropper::pointCloudCallback, this);

    cloudOutPublisher_ =
        nh_.advertise<PointCloud>(out_topic_, 1);

    server_ptr_.reset( new dynamic_reconfigure::Server<FilterConfig>(pnh_) );
    server_ptr_->setCallback(
      boost::bind(&PointCloudCropper::filterParamsCb, this, _1, _2));

    //stop pcl from spamming
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
  }

  void PointCloudCropper::filterParamsCb(const FilterConfig& config, uint32_t level)
  {
    min_elevation_ = config.min_elevation;
    max_elevation_ = config.max_elevation;

    inlier_radius_ = config.inlier_radius;
    min_neighbours_num_ = config.min_neighbours_num;

    leaf_size_ = config.leaf_size;
  }

  void PointCloudCropper::pointCloudCallback(const PointCloud::ConstPtr& cloudMsg)
  {
    //ROS_INFO("cb");
    PointCloud::Ptr withoutNan(new PointCloud);
    // remove NaN
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloudMsg, *withoutNan, indices);
    //ROS_INFO("withoutNan");

     PointCloud::Ptr voxelized(new PointCloud);
     //apply VoxelGrid filter
     pcl::VoxelGrid <pcl::PointXYZ> sor;
     sor.setInputCloud(withoutNan);
     sor.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
     sor.filter(*voxelized);

    PointCloud::Ptr transformed(new PointCloud);
    try
    {
      listener_.waitForTransform(
          reference_frame_,
          cloudMsg->header.frame_id,
          ros::Time(cloudMsg->header.stamp),
          ros::Duration(0.1));
      bool cool;
      cool = pcl_ros::transformPointCloud(reference_frame_, *voxelized,
          *transformed, listener_);
      if (!cool)
        return;
    }
    catch (const tf::TransformException& ex)
    {
      ROS_ERROR("[%s] %s", nodeName_.c_str(), ex.what());
      return;
    }
    //ROS_INFO("transformed");

    PointCloud::Ptr withoutOutliers(new PointCloud);
    // apply outlier removal (takes a lot of time)
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outlier(true);
    outlier.setInputCloud(transformed);
    outlier.setRadiusSearch(inlier_radius_);
    outlier.setMinNeighborsInRadius(min_neighbours_num_);
    outlier.filter(*withoutOutliers);

    //ROS_INFO("withoutOutliers");

    PointCloud::Ptr pointCloudMsgOut(new PointCloud);
    // filter points
    BOOST_FOREACH(const pcl::PointXYZ& pt, withoutOutliers->points)
    {
      if (pt.z > min_elevation_ && pt.z < max_elevation_)
      {
        pointCloudMsgOut->points.push_back(pt);
      }
    }

    //ROS_INFO("cropped");
    // publish final cloud
    pointCloudMsgOut->header.stamp = cloudMsg->header.stamp;
    pointCloudMsgOut->header.frame_id = reference_frame_;

    cloudOutPublisher_.publish(pointCloudMsgOut);
  }

}  // namespace pandora_costmap

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "cloud_cropper", ros::init_options::NoSigintHandler);
  pandora_costmap::PointCloudCropper cropper;
  ros::spin();
  return 0;
}
