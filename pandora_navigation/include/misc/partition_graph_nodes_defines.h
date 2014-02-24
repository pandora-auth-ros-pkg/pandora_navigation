#ifndef PARTITION_GRAPH_NODES_DEFINES_H
#define PARTITION_GRAPH_NODES_DEFINES_H


//TODO:edit these

	#define WALL_DISTANCE		22   //!< The safe distance from wall in general
	#define CLOSE_TO_WALL 		WALL_DISTANCE  //!< The safe distance from wall in partition graph
	//~ #ifdef ONE_RRT					//!< Distance ,in various methods, that the robot should keep from the walls
		//~ #define WALL_DISTANCE		20   //!< The safe distance from wall in one RRT
	//~ #elif defined MULTIPLE_RRTS
		//~ #define WALL_DISTANCE		20   //!< The safe distance from wall in RRTs
	//~ #elif defined VISIBILITY_GRAPH
		//~ #define WALL_DISTANCE		25   //!< The safe distance from wall in Visibility Graph
	//~ #else
		
	//~ #endif
	//~ #define WALL_DISTANCE_SIMPLE_RRT 21  //!< The safe distance from wall in simple RRT


	
	//---D---/
	#define DECOMP_STEP			10								//!< Decomposision step lenght in pixels (the square side)
	#define DECOMP_STEP_DIAG	sqrt(2*pow(DECOMP_STEP,2))+1	//!< Decomposision square diagonal
	#define DECOMP_TARGET_DIST 	20								//!< Decomposision target and goal distance to make neighbors


#endif

