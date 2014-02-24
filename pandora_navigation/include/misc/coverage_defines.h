#ifndef COVERAGE_DEFINES_H
#define COVERAGE_DEFINES_H

	//---C---//
	#define CAMERA_GAIN 			0.05 	//!<Camera coefficient
	#define COVERAGE_GAIN 			0.8		//!< Coverage patch density
	#define CO2_GAIN 				0.1		//!< CO2 sensor coefficient


	//---E---//
	#define EMPTY_THRESHOLD		130			//!< Map-wise unexplored area minimum threshold 


	//---G---//
	#define GLOBAL_GAIN			0.2			//!< Coefficient for all robot's sensors


	//---I---//
	#define ID_THERMAL_TPA		0					//!< TPA thermal sensor ID
	#define ID_CAMERA			1					//!< Camera ID
	#define ID_CO2				2					//!< CO2 sensor ID
	#define ID_SOUND			3					//!< sound sensor ID
	#define ID_THERMAL_MLX		4					//!< MLX thermal sensor ID
	#define ID_ROBOT			0					//!<  robot's main body ID
	#define ID_HEAD				1					//!<  robot's head ID
	

	//---M---//
	#ifndef MAP_SIZE_H
	#define MAP_SIZE_H
		#define MAP_HEIGHT				4096		//!< Map height
		#define MAP_WIDTH				4096		//!< Map width
		#define MAP_SIZE				4096 		//!< Maximum map size
		#define	START_X					MAP_HEIGHT/2		//!< Start robot X coordinate in map
		#define	START_Y 				MAP_HEIGHT/2		//!< Start robot Y coordinate in map
	#endif

	//---R---//
	#define ROBOT_SENSORS		3		//!< The number of distince robot sensors


	//---S---//
	#define SOUND_GAIN		0.1				//!< Sound sensor coefficient 


	//---T---//
	#define THERMAL_TPA_GAIN	0.2		//!< TPA thermal sensor coefficient
	#define THERMAL_MLX_GAIN	0.5		//!< MLX termal sensor coefficient



	#define OCGD 					 0.02
	#define D_PI            		3.141592654		//!< PI
	#define D_PI_DOUBLE     		6.283185308		//!< 2PI


#endif

