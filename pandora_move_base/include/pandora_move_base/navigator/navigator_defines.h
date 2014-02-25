
#ifndef DEFINES_NAVIGATOR_H
#define DEFINES_NAVIGATOR_H

	#include "geometry_msgs/PoseStamped.h"
	#include "base_local_planner.h"
	#include "pandora_navigation_common/misc/pixelcoords.h"
	#include "pandora_navigation_common/map/transformation.h"
	#include "pandora_navigation_common/misc/navigator/ir_scans.h"
	#include "pandora_navigation_common/misc/navigator/laser_scans.h"
	#include "pandora_navigation_common/misc/navigator/sonar_scan.h"

	//~ #define FLOW
	//~ #define FLOW_CORRECTION


	#define D_LASERANGRAD 			4.712389	//!< Angle field of laser
	#define DISTANCE_TO_APPROACH 	20			//!< Distance in pixels for a subgoal to be considered visited
	//~ #define DISTANCE_TO_APPROACH 	23			//!< Distance in pixels for a subgoal to be considered visited
	#define EXTRA_OBSTACLE 			0.2 		//!< Extra virtual obstacle for obstacle avoidance
	#define GOAL_DIST 		  		150			//!< Approach distance in pixels

	//---I---//
	#define INFINITY_DISTANCE   	100000000000.0		//!< A very big float number 

	//---L---//
	#define LINEAR_MAX 				1.00		//!< Maximum linear speed
	#define LINEAR_COEF 			0.001		//!< Coefficient for linear correction 
	#define LASER_RAYS     			726			//!< Number of laser rays

	//---M---//
	#ifndef MAP_SIZE_H
	#define MAP_SIZE_H
		#define MAP_HEIGHT				4096		//!< Map height
		#define MAP_WIDTH				4096		//!< Map width
		#define MAP_SIZE				4096 		//!< Maximum map size
		#define	START_X					MAP_HEIGHT/2		//!< Start robot X coordinate in map
		#define	START_Y 				MAP_HEIGHT/2		//!< Start robot Y coordinate in map
	#endif
	//---O---//
	#define OBST_AVOID_COEF 		0.03 		//!< General obstacle avoidance coefficient

	//---R---//
	#define ROTATIONAL_MAX 			1.00		//!< Maximum rotational speed
	#define ROTATIONAL_COEF 		0.01		//!< Coefficient for rotational speed

	//---S---//


	//---T---//
	#define TIME_TO_LINES 			3000				//!< Time in ms for lines correction

	#define D_LASERMAX              9.5			//!< Maximum laser measurement
	#define D_PI            		3.141592654	//!< Definition of PI
#endif
