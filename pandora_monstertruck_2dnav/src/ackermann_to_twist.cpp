/*********************************************************************
*
* software license agreement (bsd license)
*
*  copyright (c) 2016, p.a.n.d.o.r.a. team.
*  all rights reserved.
*
*  redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * neither the name of the p.a.n.d.o.r.a. team nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  this software is provided by the copyright holders and contributors
*  "as is" and any express or implied warranties, including, but not
*  limited to, the implied warranties of merchantability and fitness
*  for a particular purpose are disclaimed. in no event shall the
*  copyright owner or contributors be liable for any direct, indirect,
*  incidental, special, exemplary, or consequential damages (including,
*  but not limited to, procurement of substitute goods or services;
*  loss of use, data, or profits; or business interruption) however
*  caused and on any theory of liability, whether in contract, strict
*  liability, or tort (including negligence or otherwise) arising in
*  any way out of the use of this software, even if advised of the
*  possibility of such damage.
*
* author:  George Kouros
*********************************************************************/

#include "pandora_monstertruck_2dnav/ackermann_to_twist.h"
#include <math.h>

namespace monstertruck_2dnav
{
  Ackermann2Twist::Ackermann2Twist()
  {
    bool publishAckermann, publishTwist;

    nodeHandle_.param<bool>("ackermann_to_twist/publish_ackermann", publishAckermann, true);
    nodeHandle_.param<bool>("ackermann_to_twist/publish_twist", publishTwist, true);
    nodeHandle_.param<double>("ackermann_to_twist/wheelbase", wheelbase_, 0.32);

    if (publishTwist)
    {
      ackermannSub_ = nodeHandle_.subscribe(
        "/ackermann_cmd",
        10,
        &Ackermann2Twist::ackermannDriveCallback,
        this);

      twistPub_ = nodeHandle_.advertise<geometry_msgs::Twist>(
        "/ackermann_to_twist/cmd_vel", 10);
    }

    if (publishAckermann)
    {
      twistSub_ = nodeHandle_.subscribe(
        "/cmd_vel",
        10,
        &Ackermann2Twist::twistCallback,
        this);

      ackermannPub_ = nodeHandle_.advertise<ackermann_msgs::AckermannDrive>(
        "/twist_to_ackermann/ackermann_cmd", 10);
    }
  }

  Ackermann2Twist::~Ackermann2Twist()
  {
  }

  void Ackermann2Twist::ackermannDriveCallback(
        const ackermann_msgs::AckermannDriveConstPtr& msg)
  {
    ROS_INFO("Received msg");
    geometry_msgs::Twist twist;

    twist.linear.x = msg->speed;

    if (fabs(msg->steering_angle) > 0.01)  // w=v/R
      twist.angular.z =
        twist.linear.x /
        sqrt(pow(wheelbase_ / 2, 2) +
             pow(wheelbase_, 2) / tan(msg->steering_angle));
    else
      twist.angular.z = 0;

    twistPub_.publish(twist);
  }


  void Ackermann2Twist::twistCallback(const geometry_msgs::TwistConstPtr& msg)
  {
    ackermann_msgs::AckermannDrive ackermannDrive;

    ackermannDrive.speed = msg->linear.x;

    if (fabs(msg->angular.z) > 0.01)
      ackermannDrive.steering_angle =
        (pow(msg->linear.x / msg->angular.z, 2) - pow(wheelbase_/2, 2)) /
        pow(wheelbase_, 2);
    else
      ackermannDrive.steering_angle = 0;

    ackermannPub_.publish(ackermannDrive);
  }

}  // namespace monstertruck_2dnav
