/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, P.A.N.D.O.R.A. Team.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the P.A.N.D.O.R.A. Team nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Manos Tsardoulias
* Author: Aris Thallas
*********************************************************************/

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
