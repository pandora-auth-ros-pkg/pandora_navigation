#ifndef GLOBAL_PATH_PLANNER_H
#define GLOBAL_PATH_PLANNER_H


#include "map/map_attributes.h"
#include "navigation/path_planner/base_global_planner.h"
#include "geometry_msgs/PoseStamped.h"
#include "misc/pixelcoords.h"
#include "nodes/nodes_vector.h"

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


