#include "target_selector.h"
#include "ros/ros.h"

#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Int16.h>
#include <data_fusion_communications/GetVictimsAction.h>
#include "geometry_msgs/PoseStamped.h"


typedef actionlib::SimpleActionClient<data_fusion_communications::GetVictimsAction> VictimsClient;

class VictimTargetSelector: public TargetSelector{
	
protected:
	
	ros::NodeHandle _nh;
	//~ std::vector<geometry_msgs::PoseStamped> _victimsVector;
	
private:
	
	VictimsClient _getVictimsActionClient;
	ros::Publisher _selectedTargetIndexPublisher;
	
	geometry_msgs::PoseStamped chooseVictim( std::vector<geometry_msgs::PoseStamped>& victimsVector );
	
	
public:
	
	//~ VictimTargetSelector(void) {}
	VictimTargetSelector(MapAttributes& mapAttr, Coverage& cov);
	void selectTarget(PixelCoords* target);
	//~ void selectTarget(geometry_msgs::PoseStamped* target);
	bool selectTarget(geometry_msgs::PoseStamped* target);
	
};
