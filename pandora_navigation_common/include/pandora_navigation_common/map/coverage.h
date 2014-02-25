 /** 
  * File Description coverage (Header file) - Implements the coverage functionality
  * Contents:
		* class Coverage
  * Author: Navigation team
  */

#ifndef COVERAGE_NAVIGATION_H
#define COVERAGE_NAVIGATION_H

#include "vector"
#include "cmath"
#include "pandora_navigation_common/misc/sensor.h"
#include "pandora_navigation_common/misc/coverage_defines.h"
#include "ros/ros.h"
//~ #include "functions.h"


/*! \class Coverage
    \brief The class that manipulates the coverage patch 
*/

class Coverage{
public:

		unsigned char **robotPatch;            	//!< The basic robot patch
		unsigned char **headPatch;				//!the basic head patch
		unsigned char **coverage;				//!the basic head patch

		
		unsigned int xstart;              		//!< The center of the patch , x coordinate
        unsigned int ystart;              		//!< The center of the patch , y coordinate
        
		std::vector<Sensor> sensors;      		//!< Vector of robot's sensors
		
		int robotMaxX;                         	//!< Shape of robot patch coverage , x coordinate
        int robotMaxY;                         	//!< Shape of robot patch coverage , y coordinate
		int headMaxX;                         	//!< Shape of robot patch coverage , x coordinate
        int headMaxY;                         	//!< Shape of robot patch coverage , y coordinate
        
        /**
		@brief Void constructor
		@return Void
		**/
		Coverage(void);
		
		Coverage(int Height, int Width);
		
		/**
		@brief Adds a Sensor in the coverage patch
		@param sensorID [char] : THe sensor ID
		@param angle [float] : The angle of the sensor acording to the robot
		@param DOV [float] : Maximum distance of measurement
		@param AOV [float] : Angle of view
		@param position [char] : 0 for body, 1 for head
		@return Void
		**/
		void addSENSOR(char sensorID,float angle,float DOV,float AOV, char position);
		
		/**
		@brief Constructs the robot patch
		@return Void
		**/
		void fixRobotPatch(void);
		
		/**
		@brief Constructs the head patch
		@return Void
		**/
		void fixHeadPatch(void);
		
		/**
		@brief Flushes coverage
		@return Void
		**/
		void flush(void);
		
		/**
		@brief Patches the map at a specific point
		@param x [int]: The x coordinate of the robot
		@param y [int]: The y coordinate of the robot
		@param theta [float]: The angle of the robot
		@param Coverage [char **]: The coverage map
		@param map [unsigned char **]: The occupancy grid map
		@param dX [int]: The head's translation by x axis from the robot's body
		@param dY [int]: The head's translation by y axis from the robot's body
		@param dY [int]: The head's rotation according to robot's angle
		@return Void
		**/
		//void patchMapAt(int x,int y,float theta,unsigned char **Coverage,unsigned char **map,int dX,int dY,float dTheta);
		void patchMapAt(int x,int y,float theta,unsigned char **map,int dX,int dY,float dTheta);
		
};


#endif
