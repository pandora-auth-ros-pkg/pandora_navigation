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

#ifndef PANDORA_MONSTERTRUCK_2DNAV_ACKERMANN_TO_TWIST_H
#define PANDORA_MONSTERTRUCK_2DNAV_ACKERMANN_TO_TWIST_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <ackermann_msgs/AckermannDrive.h>

namespace monstertruck_2dnav
{

class Ackermann2Twist
{
  public:
    /**
    @brief Default Constructor
     **/
    Ackermann2Twist();

    /**
    @brief Default Destructor
    **/
    ~Ackermann2Twist();

    /**
    @brief AckermannDrive command callback that publishes Twist msgs
    @return void
    **/
    void ackermannDriveCallback(
      const ackermann_msgs::AckermannDriveConstPtr& msg);

    /**
    @brief Twist command callback that publishes AckermannDrive msgs
    @return void
    **/
    void twistCallback(const geometry_msgs::TwistConstPtr& msg);

  private:
    ros::NodeHandle nodeHandle_;  //!< ros node handle

    ros::Subscriber ackermannSub_;  //!< AckermannDrive subscriber
    ros::Subscriber twistSub_;  //!< Twist subscriber

    ros::Publisher ackermannPub_;  //!< AckermannDrive publisher
    ros::Publisher twistPub_;  //!< Twist publisher

    double wheelbase_;  //!< the wheel separation length of the robot
};  // class Ackermann2Twist

}  // namespace monstertruck_2dnav

#endif  // PANDORA_MONSTERTRUCK_2DNAV_ACKERMANN_TO_TWIST_H
