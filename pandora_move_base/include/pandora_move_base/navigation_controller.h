#ifndef NAVIGATION_CONTROLLER_H
#define NAVIGATION_CONTROLLER_H

#include "pandora_move_base/navigation_controller_defines.h"

class NavigationController{
	
public:
	LaserInfo lsr;
	NavigationController(void);
	
private:
	MapUpdater _mapUpdater;
	PandoraNavigator navigator;
	PandoraPathPlanner pandoraPathPlanner;
	MapAttributes& mapAttributes;
	
	ros::Subscriber _sonarSubscriber;
	ros::Subscriber _headSonarSubscriber;
	ros::Subscriber _laserScansSubscriber;
	ros::Subscriber _compassSubscriber;
	// Path
	//~ std::vector<PixelCoords> pixelPlan;				//!< Vector with path to target
	PixelCoords pixelGoal;							//!<Target
	bool performInitial;
	geometry_msgs::PoseStamped poseStampedGoal;
	std::vector<geometry_msgs::PoseStamped> poseStampedPlan;
	std::vector<geometry_msgs::PoseStamped>* poseStampedPlanPointer;
	
	LaserScan scan;				//!< The current laser scan
	SonarScan sonars;    		//!< The current sonar scan
	IrScan irs;   				//!< The current IR scan
	float compassPitch;		//!< Holds the compass pitch
	
	
	
	float expectedDx;
	float expectedDy;
	float expectedDtheta;
	int stuckCounter;
	
	bool stuckReset;
	Transformation expectedRobotPose;
	
	
	bool calcPath( const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& poseStampedPlan );
	void sendSpeedsToMotors(geometry_msgs::Twist twistVelocities);
	
	int moveToFinalTarget(std::vector<geometry_msgs::PoseStamped>& poseStampedPlan);
	int moveToTargetWithSubGoals(std::vector<geometry_msgs::PoseStamped>& poseStampedPlan);
	int moveToTargetRecalculate(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& poseStampedPlan);
	int moveToTargetRecalculate_stuckAvoidance(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& poseStampedPlan);
	
	bool performInitialTurn(void);
	int parkRobot(float theta);
	float calcParkingAngle(float parkOrientation);
	
	void stopMotors(void);
	void rotateRobotToUnstuck(void);
	void moveRobotToUnstuck(void);
	
	
	//~ void updateRobotPose(const slam_communications::robotPoseMsg& msg);
	//~ void updateIRs(const controllers_and_sensors_communications::irMsg& msg);
	void updateSonars(const sensor_msgs::Range& msg);
	void updateLaserScans(const sensor_msgs::LaserScanConstPtr& msg);
	void serveCompassMessage(const controllers_and_sensors_communications::compassMsg& msg);
	
	PixelCoords transformPoseStamped2PixelCoords(geometry_msgs::PoseStamped point);
	geometry_msgs::PoseStamped transformPixelCoords2PoseStamped(PixelCoords point);
	void transformTwist2Velocities( geometry_msgs::Twist twist, float* linear, float* rotational);
	
	bool sendMap(pandora_navigation_communications::navigationMapSrv::Request &req,pandora_navigation_communications::navigationMapSrv::Response &res);
	
	bool checkPreempt(void);
	
	void publishPath(const ros::TimerEvent&); 
	
protected:
	ros::NodeHandle nh_;
	
	
	ros::Publisher _velocityPublisher;
	ros::Publisher _pathPublisher;
	ros::ServiceServer _mapService;
	
	ros::Timer _pathStreamingTimer;
	
	// NodeHandle instance must be created before this line. Otherwise strange error may occur.
	actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> moveBaseActionServer_; 
	actionlib::SimpleActionServer<pandora_navigation_communications::InitialTurnAction> initialTurnActionServer_; 
	actionlib::SimpleActionServer<pandora_navigation_communications::RobotStuckAction> robotStuckActionServer_; 
	std::string action_name_;
	
	void moveBaseActionCallBack(const move_base_msgs::MoveBaseGoalConstPtr &goal);
	void initialTurnActionCallBack(const pandora_navigation_communications::InitialTurnGoalConstPtr &goal);
	void robotStuckActionCallBack(const pandora_navigation_communications::RobotStuckGoalConstPtr &goal);
	
};

#endif

