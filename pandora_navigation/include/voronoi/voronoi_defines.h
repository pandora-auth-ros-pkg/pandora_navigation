#ifndef VORONOI_DEFINES_H
#define VORONOI_DEFINES_H
		
		
	//---E---//
		#define EMPTY_THRESHOLD		130			//!< Map-wise unexplored area minimum threshold
		
		
	//---M---//
		#define MIN_THRESHOLD		18  	//!< Minimum accepted distance from wall for voronoi creation 


	#ifndef MAP_SIZE_H
	#define MAP_SIZE_H
		#define MAP_HEIGHT				4096		//!< Map height
		#define MAP_WIDTH				4096		//!< Map width
		#define MAP_SIZE				4096 		//!< Maximum map size
		#define	START_X					MAP_HEIGHT/2		//!< Start robot X coordinate in map
		#define	START_Y 				MAP_HEIGHT/2		//!< Start robot Y coordinate in map
	#endif
		
	//---V---//
		#define VORONOIPARENTDISTANCE	5		//!< Minimum parent to parent distance at forming the GVD
		#define VORONOITHRESHOLD		20		//!< Steps of voronoi in extended mode
		
		
	//---W---//
		#define WALL_THRESHOLD			120		//!< Map-wise maximum threshold considered wall
		
#endif

