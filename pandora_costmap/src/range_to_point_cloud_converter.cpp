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
* Author:  Evangelos Apostolidis
*********************************************************************/

#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/PointCloud2.h>

namespace pandora_navigation
{
  class RangeToPCConverter
  {
    private:
      ros::NodeHandle nodeHandle_;
      ros::Subscriber rangeSubscriber_;
      ros::Publisher rangePublisher_;
      std::string rangeTopic_;
      double angleResolution_;
      double maxRange_;

      void rangeCallback(const sensor_msgs::RangeConstPtr msg);
    public:
      RangeToPCConverter();
      ~RangeToPCConverter();
  };

  RangeToPCConverter::RangeToPCConverter()
  {
    nodeHandle_.param("/range_to_point_cloud_converter/angle_resolution", angleResolution_, 0.045);
    nodeHandle_.param("/range_to_point_cloud_converter/max_range", maxRange_, 2.0);
    rangeTopic_ = "/sensors/range";
    rangeSubscriber_ =
      nodeHandle_.subscribe(
        rangeTopic_,
        1,
        &RangeToPCConverter::rangeCallback,
        this);

    rangePublisher_ =
      nodeHandle_.advertise<sensor_msgs::PointCloud2>(
        rangeTopic_ + "/point_cloud", 1, true);
  }

  RangeToPCConverter::~RangeToPCConverter()
  {
  }

  void RangeToPCConverter::rangeCallback(
    const sensor_msgs::RangeConstPtr msg)
  {
    float fieldOfView = msg->field_of_view;
    float range = msg->range;

    if (range > msg->min_range && range < maxRange_ && range < msg->max_range)
    {
      sensor_msgs::PointCloud2 cloud;
      cloud.header.stamp = msg->header.stamp;
      cloud.header.frame_id = msg->header.frame_id;
      cloud.height = 1;
      cloud.width = 0;
      cloud.is_bigendian = true;
      cloud.is_dense = true;
      cloud.point_step = 12;

      sensor_msgs::PointField field;
      field.name = "x";
      field.offset = 0;
      field.datatype = 7;
      field.count = 1;
      cloud.fields.push_back(field);
      field.name = "y";
      field.offset = 4;
      cloud.fields.push_back(field);
      field.name = "z";
      field.offset = 8;
      cloud.fields.push_back(field);

      float step = fieldOfView * 3.14 / 180.0 / 2;

      for (float theta = - step;
        theta < step;
        theta = theta + angleResolution_)
      {
        for (float phi = 1.57 - step * cos(asin(theta / step));
          phi < 1.57 + step * cos(asin(theta / step));
          phi = phi + angleResolution_)
        {
          float x, y, z;
          uint8_t *xData, *yData, *zData;
          x = range * cos(theta) * sin(phi);
          y = range * sin(theta) * sin(phi);
          z = range * cos(phi);

          xData = reinterpret_cast<uint8_t*>(&x);
          yData = reinterpret_cast<uint8_t*>(&y);
          zData = reinterpret_cast<uint8_t*>(&z);

          // x
          cloud.data.push_back(xData[0]);
          cloud.data.push_back(xData[1]);
          cloud.data.push_back(xData[2]);
          cloud.data.push_back(xData[3]);
          // y
          cloud.data.push_back(yData[0]);
          cloud.data.push_back(yData[1]);
          cloud.data.push_back(yData[2]);
          cloud.data.push_back(yData[3]);
          // z
          cloud.data.push_back(zData[0]);
          cloud.data.push_back(zData[1]);
          cloud.data.push_back(zData[2]);
          cloud.data.push_back(zData[3]);
          cloud.width++;
        }
      }
      cloud.row_step = cloud.point_step * cloud.width;
      rangePublisher_.publish(cloud);
    }
  }
}  // namespace pandora_navigation

int main(int argc, char **argv)
{
  ros::init(argc, argv, "range_to_point_cloud_converter");
  pandora_navigation::RangeToPCConverter rangeToPCConverter;
  ros::spin();
}
