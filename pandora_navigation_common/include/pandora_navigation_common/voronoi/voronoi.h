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

#ifndef VORONOI_H
#define VORONOI_H


#include "voronoi_defines.h"
#include "pandora_navigation_common/misc/pixelcoords.h"
#include "pandora_navigation_common/map/map_attributes.h"
#include "vector"

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

//TODO: create new constructor with bool argument for partitionGraph
//TODO: new function for brushCell calculation

class Voronoi{
	
private:
	
	//~ **bool voronoi;
	//~ int mapHeight,mapWidth;
	
	//computes voronoi and brushCell
	//void computeVoronoi(void);
	//void computeVoronoi(unsigned char **map, bool **isOnVoronoi, MapInfo info,unsigned int xMax, unsigned int xMin,unsigned int yMax,unsigned int yMin,float **brushCell,PixelCoords **parent);
	void computeVoronoi(unsigned char **map,int mapHeight, int mapWidth,unsigned int xMax, unsigned int xMin,unsigned int yMax,unsigned int yMin,float **brushCell);
	
	
	//Trims the voronoi edges by "contamination" concept
	void trimVoronoiEdges(int mapHeight, int mapWidth);
	//~ void trimVoronoiEdges(bool **voronoi, MapInfo &info);
	
	//Checks if a Point is a voronoi's edge
	//bool isEdge(int i, int j);
	bool isEdge(int i, int j, int mpaHeight, int mapWigth);
	//~ bool isEdge(bool **voronoi, int i, int j, MapInfo &info);
	
	
	bool performThinningIteration(unsigned char **img,unsigned char **img2,int width,int height);
	bool performPruningIteration(unsigned char **img,unsigned char **img2,int width,int height);
	void copyElements(unsigned char **img,unsigned char **img2,int width,int height);
	bool performFinalPruningIteration(unsigned char **img,unsigned char **img2,int width,int height);
	
	
public:
	
	template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
		
		for(int ii=0;ii<mapAttributes->getHeight();ii++){
			for(int jj=0;jj<mapAttributes->getWidth();jj++){
				
				ar & voronoi[ii][jj];
			
		}
	}
		
		
		
		
    }
	
	
	bool** voronoi;
	PixelCoords **parent;
	float** brushCell;
	
	MapAttributes* mapAttributes;
	
	//voronoi constructor. TODO: maybe add void constructor
	//Voronoi(void){}
	//~ void Voronoi(const MapAttributes* mapAttr);
	//Voronoi(const MapAttributes* mapAttr, bool brush=false);
	Voronoi(MapAttributes* mapAttr, bool brush=false);
	
	
	void fixVoronoi(void);
	void computeBrushCell(void);
	//~ void computeBrushCell(unsigned char **map,int mapHeight, int mapWidth,unsigned int xMax, unsigned int xMin,unsigned int yMax,unsigned int yMin);
	
	
};


#endif

	//~ TODO:remove these and edit functions
	//~ void setMapHeight(int height) { mapHeight = height };
	//~ void setMapWidth(int width) { mapWidth = width };
	//~ int getMapHeight(void) { return height };
	//~ int getMapWidth(void) { return width };
