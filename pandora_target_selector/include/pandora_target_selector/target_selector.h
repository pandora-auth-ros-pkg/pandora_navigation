#ifndef TARGET_SELECTOR_H
#define TARGET_SELECTOR_H

//~ #include "slam_communications/robotPoseMsg.h"

#include "pandora_navigation_common/map/map_attributes.h"
#include "pandora_navigation_common/map/coverage.h"
#include "pandora_navigation_common/map/map_updater.h"
#include "pandora_navigation_common/misc/pixelcoords.h"
#include "geometry_msgs/PoseStamped.h"
#include "pandora_navigation_communications/navigationMapSrv.h"

class TargetSelector{
	

public:

	//~ TargetSelector(void);
	TargetSelector(MapAttributes& mapAttr, Coverage& cov);
	
	//~ virtual PixelCoords selectTarget(void)=0;
	virtual void selectTarget(PixelCoords* target)=0;
	
protected:
	
	//~ void patchCoverage(const slam_communications::robotPoseMsg& msg);
	
	//~ ros::NodeHandle _nh;
	
	
	MapAttributes& mapAttributes;
	Coverage& coverage;
	//~ ros::Subscriber robotPoseSubscriber;	
	//~ MapUpdater _mapUpdater;
	
	PixelCoords transformPoseStamped2PixelCoords(geometry_msgs::PoseStamped point);
	geometry_msgs::PoseStamped transformPixelCoords2PoseStamped(PixelCoords point);
};

#endif

