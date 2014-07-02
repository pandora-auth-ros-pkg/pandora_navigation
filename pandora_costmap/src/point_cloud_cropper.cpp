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

#include "pandora_costmap/point_cloud_cropper.h"

namespace pandora_costmap {

  PointCloudCropper::PointCloudCropper() : nh_(), pnh_("~")
  {
    std::string inTopic, outTopic; 
    
    pnh_.param("min_frame_id", minFrame_, std::string("base_stabilized"));
    pnh_.param("max_frame_id", maxFrame_, std::string("kinect_link"));

    pnh_.param("in_topic", inTopic, std::string("kinect/point_cloud"));
    pnh_.param("out_topic", outTopic, std::string("costmap/cropped_cloud"));

    cloudInSubscriber_ =
        nh_.subscribe(inTopic, 1, &PointCloudCropper::pointCloudCallback, this);

    cloudOutPublisher_ =
        nh_.advertise<PointCloud>(outTopic, 1);

    //stop pcl from spamming
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
  }

  void PointCloudCropper::pointCloudCallback(const PointCloud::ConstPtr& cloudMsg)
  {
    // find elevation between frames
    tf::StampedTransform tfTransform;
    float elevation;
    try {
      listener_.waitForTransform(minFrame_,
          maxFrame_, ros::Time(0), ros::Duration(0.1));
      listener_.lookupTransform(minFrame_,
          maxFrame_, ros::Time(0), tfTransform);

      elevation = tfTransform.getOrigin()[2];
    }
    catch (tf::TransformException ex) {
      ROS_ERROR("[cloud_cropper] %s", ex.what());
      return;
    }
    
    // cloud holders
    PointCloud::Ptr pointCloudMin(new PointCloud);
    PointCloud::Ptr pointCloudOut(new PointCloud);
    PointCloud::Ptr pointCloudMsgOut(new PointCloud);
    PointCloud::Ptr pointCloudFiltered(new PointCloud);
    PointCloud::Ptr pointCloudFinal(new PointCloud);

    // transform cloud
    try {
      listener_.waitForTransform(minFrame_,
          cloudMsg->header.frame_id,
          ros::Time(cloudMsg->header.stamp),
          ros::Duration(0.1));
      pcl_ros::transformPointCloud(minFrame_, *cloudMsg, *pointCloudMin, listener_);
    }
    catch (tf::TransformException ex) {
      ROS_ERROR("[cloud_cropper] %s", ex.what());
      return;
    }
    
    // remove NaN
    std::vector<int> index;
    pcl::removeNaNFromPointCloud(*pointCloudMin, *pointCloudMin, index);

    // filter points
    BOOST_FOREACH(const pcl::PointXYZ& pt, pointCloudMin->points)
    {
      if (pt.z > 0 && pt.z < elevation) {
        pointCloudMsgOut->points.push_back(pt);
      }
    }

    // apply VoxelGrid filter
    pcl::VoxelGrid <pcl::PointXYZ> sor;
    sor.setInputCloud (pointCloudMsgOut);
    float leafSize = 0.01;
    sor.setLeafSize (leafSize, leafSize, leafSize);
    sor.filter (*pointCloudFiltered);
    
    // apply outlier removal (takes a lot of time)
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outlier (true);
    outlier.setInputCloud(pointCloudFiltered);
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outlieri(true);
    outlier.setRadiusSearch(0.3);
    outlier.setMinNeighborsInRadius(5);
    outlier.filter(*pointCloudFinal);

    // publish final cloud 
    pointCloudFinal->header.stamp = cloudMsg->header.stamp;
    pointCloudFinal->header.frame_id = minFrame_;

    cloudOutPublisher_.publish(pointCloudFinal);
  }

}  // namespace pandora_costmap


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "cloud_cropper", ros::init_options::NoSigintHandler);

  pandora_costmap::PointCloudCropper cropper;

  ros::spin();
  
  return 0;
}
