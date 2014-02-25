#ifndef NAVIGATION_MAP_ATTRIBUTES
#define NAVIGATION_MAP_ATTRIBUTES


#include "transformation.h"
#include <vector>


class MapAttributes{
	
public:
	
	int height;     //!< The map's height
	int width;      //!< The map's width
	
	

	
	unsigned char** map;		//!< The map matrix
	
	int prevxMax;				//!< The previous limits of the map - Maximum X
	int prevxMin;				//!< The previous limits of the map - Minimum X
	int prevyMax;				//!< The previous limits of the map - Maximum Y
	int prevyMin;				//!< The previous limits of the map - Minimum Y
	
	Transformation robotPose;
	std::vector<Transformation> robotTrajectory;
	
	//these are probably not needed
	int xMax;					//!< The limits of the map - Maximum X
	int xMin;					//!< The limits of the map - Minimum X
	int yMax;					//!< The limits of the map - Maximum Y
	int yMin;					//!< The limits of the map - Minimum Y
	//
	
	/**
	@brief Constructor
	@param height [int]	:The map's height
	@param width [int]	:The map's width
	@return void
	 **/
	MapAttributes(int height, int width);
	
	/**
	@brief Void constructor
	@param void
	@return void
	**/
	MapAttributes(void){
		xMax=xMin=yMax=yMin=4096/2;
	}
	
	 //maybe robotPose needs to be initialized
		   
	/**
	@brief Getter of height
	@param void
	@return void
	**/
	int getHeight() {   return height;  };  
	
	/**
	@brief  Getter of width
	@param void
	@return void
	**/
	int getWidth()  {   return width;   };
	
};


#endif
