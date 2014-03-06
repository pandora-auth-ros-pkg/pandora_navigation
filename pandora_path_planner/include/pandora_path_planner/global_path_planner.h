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
* Author: Manos Tsardoulias
* Author: Aris Thallas
*********************************************************************/

#ifndef GLOBAL_PATH_PLANNER_H
#define GLOBAL_PATH_PLANNER_H


#include "pandora_navigation_common/map/map_attributes.h"
#include "pandora_path_planner/base_global_planner.h"
#include "geometry_msgs/PoseStamped.h"
#include "pandora_navigation_common/misc/pixelcoords.h"
#include "pandora_navigation_common/nodes/nodes_vector.h"

using namespace nav_core;

class GlobalPathPlanner: public BaseGlobalPlanner {
	
private:
	
	
	
protected:
	
public:
	//maybe these are not needed
	//~ std::vector<PixelCoords> pixelPlan;				//!< Vector with path to target
	//~ PixelCoords pixelGoal;							//!<Target
	//~ geometry_msgs::PoseStamped poseStampedGoal;
	//~ std::vector<geometry_msgs::PoseStamped> poseStampedPlan;
	//~ PixelCoords transformPoseStamped2PixelCoords(geometry_msgs::PoseStamped point);
	//~ geometry_msgs::PoseStamped transformPixelCoords2PoseStamped(PixelCoords point);
	
	
	//TODO: add robot attributes
	MapAttributes* mapAttributes;
	
	//~ void slamMapCallback(const actionlib::SimpleClientGoalState& state, const slam_communications::slamMapResultConstPtr& result);
	
//public:
	
	
	//~ void NavigationController(void);
	GlobalPathPlanner(void){}
	
	GlobalPathPlanner(MapAttributes* mapAttr){}
	
	
	//move_base function
	virtual bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)=0;
	
	//targetSelector path planner to return path for all possible targets
	virtual void getTargetSelectorPaths(NodesVector* nodes)=0;
	
	
};

#endif


