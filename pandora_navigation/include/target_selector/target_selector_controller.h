#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <std_srvs/Empty.h>
#include <target_selector_communications/SelectTargetAction.h>
#include <target_selector_communications/SelectTargetResult.h>
#include "target_selector/exploration_target_selector.h"
#include "target_selector/victim_target_selector.h"
#include "target_selector/target_selector_controller_defines.h"

#include "geometry_msgs/PoseStamped.h"

#include "pandora_navigation_communications/NavigationGeotiffSrv.h"

#include <nav_msgs/OccupancyGrid.h>

class TargetSelectorController{


public:
	MapUpdater _mapUpdater;
	
	//~ ros::Subscriber robotPoseSubscriber;
	void patchCoverage(const ros::TimerEvent&);
	
	TargetSelectorController(void);
	MapAttributes& mapAttributes;
	Coverage coverage;
	
	void coverageTimerCallback(const ros::TimerEvent&);
	
	ros::Timer _coverageCallbackTimer;
	ros::Timer _coveragePatchingTimer;
	ros::Publisher _occupancyGridPublisher;
	//~ ros::Publisher takis;

private:
	
	TargetSelector* targetSelector;
	VictimTargetSelector victimTargetSelector;
	ExplorationTargetSelector explorationTargetSelector;
	ClosestUnexploredTargetSelector closestUnexploredTargetSelector;
	
	geometry_msgs::PoseStamped transformPixelCoords2PoseStamped(PixelCoords pixelCoords);
	
	bool sendMap(pandora_navigation_communications::navigationMapSrv::Request &req,pandora_navigation_communications::navigationMapSrv::Response &res);
	void makeDummyCoveragePatch(unsigned char **map,unsigned char **coverage,int x, int y,int d);
	bool flushCoverage(std_srvs::Empty::Request& rq,std_srvs::Empty::Response &rs);
	
	bool checkPreempt(void);

	
	
protected:
	ros::NodeHandle nh_;
	
	ros::ServiceServer _mapService;
	ros::ServiceServer geotiffSrv;
	
	ros::ServiceServer _flushService;
	
	// NodeHandle instance must be created before this line. Otherwise strange error may occur.
	actionlib::SimpleActionServer<target_selector_communications::SelectTargetAction> selectTargetActionServer_; 
	std::string action_name_;
	
	void selectTargetActionCallBack(const target_selector_communications::SelectTargetGoalConstPtr &goal);
	bool geotiffSericeCb(pandora_navigation_communications::NavigationGeotiffSrv::Request &req,pandora_navigation_communications::NavigationGeotiffSrv::Response &res);
	

};

