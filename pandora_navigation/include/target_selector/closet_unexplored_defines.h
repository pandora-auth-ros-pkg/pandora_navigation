#ifndef CLOSEST_UNEXPLORED_DEFINES_H
#define CLOSEST_UNEXPLORED_DEFINES_H



	#ifndef MAP_SIZE_H
	#define MAP_SIZE_H
		#define MAP_HEIGHT				4096		//!< Map height
		#define MAP_WIDTH				4096		//!< Map width
		#define MAP_SIZE				4096 		//!< Maximum map size
		#define	START_X					MAP_HEIGHT/2		//!< Start robot X coordinate in map
		#define	START_Y 				MAP_HEIGHT/2		//!< Start robot Y coordinate in map
	#endif
	
	#define WALL_THRESHOLD			120		//!< Map-wise maximum threshold considered wall 
	#define CHECK_WALL_TH			20      //!< Minimum Threshold for coverage limits
	#define MAP_LIMITS 			127		//!< Possibility of unknown space
	#define COVERAGE_LIMITS 		50		//!< Map-wise minimum threshold considered as covered




#endif

