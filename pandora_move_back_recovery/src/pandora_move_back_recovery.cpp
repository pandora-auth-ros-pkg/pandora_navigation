/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, P.A.N.D.O.R.A. Team.
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
* Author: Dimitrios Kirtsios <dimkirts@gmail.com>
*********************************************************************/

#include "pandora_move_back_recovery/pandora_move_back_recovery.h"
#include <pluginlib/class_list_macros.h>

namespace pandora_move_back_recovery
{
  // register the plugin
  PLUGINLIB_DECLARE_CLASS(pandora_move_back_recovery, PandoraMoveBackRecovery,
    pandora_move_back_recovery::PandoraMoveBackRecovery, nav_core::RecoveryBehavior)

  PandoraMoveBackRecovery::PandoraMoveBackRecovery() : global_costmap_(NULL), local_costmap_(NULL), 
  tf_(NULL), initialized_(false), world_model_(NULL)
  {}

  void PandoraMoveBackRecovery::initialize(std::string name, tf::TransformListener* tf,
  costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap)
  {
    if(!initialized_)
    {
      name_ = name;
      tf_ = tf;
      global_costmap_ = global_costmap;
      local_costmap_ = local_costmap;

      // parameter loading
      
    }
    else
    {
      ROS_ERROR("You should not call initialize twice on this object, doing nothing");
    }
  }

  void PandoraMoveBackRecovery::runBehavior()
  {
    if(!initialized_)
    {
      ROS_ERROR("This object must be initialized before runBehavior is called");
      return;
    }
  }

  PandoraMoveBackRecovery::~PandoraMoveBackRecovery()
  {}
}  // namespace pandora_move_back_recovery