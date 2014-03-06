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
