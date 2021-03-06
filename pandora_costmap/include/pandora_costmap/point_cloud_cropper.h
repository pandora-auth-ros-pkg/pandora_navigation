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

#ifndef PANDORA_COSTMAP_POINT_CLOUD_CROPPER_H
#define PANDORA_COSTMAP_POINT_CLOUD_CROPPER_H

#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <dynamic_reconfigure/server.h>

#include "pandora_costmap/FilterConfig.h"

namespace pandora_costmap
{

  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

  class PointCloudCropper
  {
   public:
    PointCloudCropper();
    void pointCloudCallback(const PointCloud::ConstPtr& cloudMsg);

    void filterParamsCb(const FilterConfig& config, uint32_t level);

   private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    std::string nodeName_;

    // The dynamic reconfigure server for general parameters
    boost::shared_ptr< dynamic_reconfigure::Server<FilterConfig> > server_ptr_;
    // The dynamic reconfigure callback type for the above server
    dynamic_reconfigure::Server<FilterConfig>::CallbackType f_type;

    ros::Subscriber cloudInSubscriber_;
    std::string in_topic_;
    ros::Publisher cloudOutPublisher_;
    std::string out_topic_;

    tf::TransformListener listener_;

    std::string reference_frame_;
    double min_elevation_;
    double max_elevation_;
    double inlier_radius_;
    int min_neighbours_num_;
    double leaf_size_;
  };

}  // namespace pandora_costmap

#endif
