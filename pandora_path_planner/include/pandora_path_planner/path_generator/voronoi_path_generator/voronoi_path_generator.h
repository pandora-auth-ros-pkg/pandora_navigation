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

#ifndef VORONOI_PATH_GENERATOR_H
#define VORONOI_PATH_GENERATOR_H

#include "pandora_path_planner/path_generator/path_generator.h"
#include "voronoi_path_generator_defines.h"

class VoronoiPathGenerator: public PathGenerator{
	
private:
	
	//~ Voronoi voronoi;
	int** field;

	PixelCoords getOptClosest(int **brushCell,bool **voronoi, unsigned char **map, int mapHeight, int mapWigth, PixelCoords position);
	
	//~ std::vector<PixelCoords> findOptPathToPoint(int **field,bool **voronoi, unsigned char **map, MapInfo info, PixelCoords position, PixelCoords destination);
	//probably needs map and map info removed
	std::vector<PixelCoords> findOptPathToPoint( bool** voronoi, unsigned char **map,int mapHeight, int mapWigth, PixelCoords position, PixelCoords destination);
	
	void drawOptField(bool **voronoi, int **field,int mapHeight, int mapWigth, PixelCoords position, PixelCoords destination);
	
	//~ void moveWithVoronoi(PixelCoords startPos, PixelCoords target, std::vector<PixelCoords>& wantedPath );
	//void moveWithVoronoi(PixelCoords startPos, PixelCoords target, std::vector<PixelCoords>& wantedPath, bool **voronoi );
	void moveWithVoronoi(PixelCoords startPos, PixelCoords target, std::vector<PixelCoords>& wantedPath );
	
public:
	
	//~ void VoronoiPathGenerator(void);
	VoronoiPathGenerator(MapAttributes* mapAttr,Voronoi *v);
	
	bool generatePath(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
	
	
};


#endif


