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

#ifndef CLOSEST_UNEXPLORED_TARGET_SELECTOR_H
#define CLOSEST_UNEXPLORED_TARGET_SELECTOR_H


#include "target_selector.h"
#include "closet_unexplored_defines.h"
//#include "target_selector/closest_unexplored_defines.h"




class ClosestUnexploredTargetSelector: public TargetSelector{
	
public:
	
	//~ unsigned char** coverage;					//!< The coverage matrix
	//~ Coverage* coverage;
	//~ MapAttributes* mapAttributes;
	PixelCoords closestCoverageUnexplored;		//!< Thw closest unexplored point
	std::vector<PixelCoords> coverageLimits;	//!< Vector Coordinates of coverage limits
	int** addField;
	int** field;
	
	
	
	
public:

	//~ ClosestUnexploredTargetSelector(void);
	ClosestUnexploredTargetSelector(MapAttributes& mapAttr, Coverage& cov);
	
	void selectTarget(PixelCoords* target);
	
	//~ std::vector<PixelCoords> findCoverageLimits(unsigned char **map,MapInfo info,Transformation robotPose,int xMin,int xMax,int yMin,int yMax,int **field,unsigned char **coverage,PixelCoords *closest,int **addField);
	//std::vector<PixelCoords> findCoverageLimits(PixelCoords *closest);
	std::vector<PixelCoords> findCoverageLimits(unsigned char **map,int mapHeight,int mpaWidth,Transformation robotPose,int xMin,int xMax,int yMin,int yMax,int **field,unsigned char **coverage,PixelCoords *closest,int **addField);
	
	//~ bool checkPointIfCloseWall(unsigned char **map, MapInfo info, PixelCoords point, int xMin, int xMax, int yMin, int yMax, int **field);
	//bool checkPointIfCloseWall(PixelCoords point, int xMin, int xMax, int yMin, int yMax, int **field);
	bool checkPointIfCloseWall(unsigned char **map,int mapHeight, int mapWidth,PixelCoords point,int xMin,int xMax,int yMin,int yMax,int **field);
	
};


#endif


