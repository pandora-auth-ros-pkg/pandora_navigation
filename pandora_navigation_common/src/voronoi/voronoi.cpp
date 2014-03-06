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

#include "pandora_navigation_common/voronoi/voronoi.h"
#include "ros/ros.h"



//~ #define FLOW

//~ void Voronoi::Voronoi(void){
//Voronoi::Voronoi(const MapAttributes* mapAttr, bool brush){
Voronoi::Voronoi(MapAttributes* mapAttr, bool brush){
	
	//mapAttributes = new MapAttributes;
	mapAttributes = mapAttr;
	
	if (brush){
		//~ voronoi = new bool*[mapAttr->getHeight()];
		parent = new PixelCoords*[mapAttr->getHeight()];
		brushCell = new float *[mapAttr->getHeight()];
		
		for(unsigned int i=0;i<mapAttr->getHeight();i++){
			
			//~ voronoi[i] = new bool[mapAttr->getWidth()];
			parent[i] = new PixelCoords[mapAttr->getWidth()];
			brushCell[i] = new float[mapAttr->getWidth()];
		}
	}
	else{
		voronoi = new bool*[mapAttr->getHeight()];
		parent = new PixelCoords*[mapAttr->getHeight()];
		brushCell = new float *[mapAttr->getHeight()];
		
		for(unsigned int i=0;i<mapAttr->getHeight();i++){
			
			voronoi[i] = new bool[mapAttr->getWidth()];
			parent[i] = new PixelCoords[mapAttr->getWidth()];
			brushCell[i] = new float[mapAttr->getWidth()];
		}
	}
	//TODO: maybe remove this.
	//~ fixVoronoi();
	
}



void Voronoi::fixVoronoi(void){
	if(mapAttributes->map[mapAttributes->robotPose.dx+START_X][mapAttributes->robotPose.dy+START_Y]<EMPTY_THRESHOLD){
		ROS_ERROR_NAMED("voronoi","[voronoi %d] Did not fix voronoi",__LINE__);
		return;
	}
	
	//~ ROS_INFO("[PLANNER %d] Inside Voronoi",__LINE__);
	
	ROS_INFO_NAMED("voronoi","[voronoi %d] Computing voronoi",__LINE__);
	computeVoronoi(mapAttributes->map, mapAttributes->getHeight(), mapAttributes->getWidth(), mapAttributes->prevxMax,mapAttributes->prevxMin,mapAttributes->prevyMax,mapAttributes->prevyMin,brushCell);
	
	
	ROS_INFO_NAMED("voronoi","[voronoi %d] Trimming voronoi edges",__LINE__);
	trimVoronoiEdges(mapAttributes->getHeight(), mapAttributes->getWidth());

	//	Perform morphological operations to voronoi
	unsigned char **img, **img2;
	unsigned int x,y;
	x=mapAttributes->prevxMax-mapAttributes->prevxMin;
	y=mapAttributes->prevyMax-mapAttributes->prevyMin;
	img=new unsigned char *[x];
	img2=new unsigned char *[x];
	for(unsigned int i=0;i<x;i++)
	{
		img[i]=new unsigned char[y];
		img2[i]=new unsigned char[y];
	}
	
	for(unsigned int i=0;i<x;i++)
		for(unsigned int j=0;j<y;j++)
			img[i][j]=img2[i][j]=voronoi[i+mapAttributes->prevxMin][j+mapAttributes->prevyMin];
			
	ROS_INFO_NAMED("voronoi","[voronoi %d] Thinning voronoi",__LINE__);
	while(performThinningIteration(img,img2,x,y)) ;
	
	ROS_INFO_NAMED("voronoi","[voronoi %d] Pruning voronoi",__LINE__);
	for(unsigned int i=0;i<10;i++)
		performPruningIteration(img,img2,x,y);
		ROS_INFO_NAMED("voronoi","[voronoi %d] Final voronoi pruning",__LINE__);
	performFinalPruningIteration(img,img2,x,y);
	
	ROS_INFO_NAMED("voronoi","[voronoi %d] Final voronoi thinning",__LINE__);
	while(performThinningIteration(img,img2,x,y)) ;
	for(unsigned int i=0;i<x;i++)
		for(unsigned int j=0;j<y;j++)
			voronoi[i+mapAttributes->prevxMin][j+mapAttributes->prevyMin]=img[i][j];
	
	
	for(unsigned int i=0;i<x;i++){
		delete [] img[i];
		delete [] img2[i];
	}
	delete [] img;
	delete [] img2;
	
	ROS_INFO_NAMED("voronoi","[voronoi %d] Voronoi is calculated",__LINE__);
}


void Voronoi::computeVoronoi(unsigned char **map,int mapHeight, int mapWidth, unsigned int xMax,unsigned int xMin,unsigned int yMax,unsigned int yMin,float **brushCell){
	
	bool stop = false;
	int currDist;
	std::vector<PixelCoords> currCheck, nextCheck;

	for(unsigned int i=xMin;i<xMax;i++){
		
		for(unsigned int j=yMin;j<yMax;j++){
			
			//~ isOnVoronoi[i][j] = false;
			voronoi[i][j] = false;
			
			if(map[i][j] < WALL_THRESHOLD) {
				brushCell[i][j] = 0;
				(parent[i][j]).setCoords(i, j);
				currCheck.push_back(PixelCoords(i, j));
			}
			else if(map[i][j] < EMPTY_THRESHOLD){
				brushCell[i][j] = 0;
				(parent[i][j]).setCoords(i, j);
				currCheck.push_back(PixelCoords(i, j));
			}
			else{
				brushCell[i][j] = -1;
				(parent[i][j]).setCoords(-1, -1);
			}
		}
	}
	unsigned int counterVoro=0;
	while(!stop){
		stop = true;
		int parentDistance = VORONOIPARENTDISTANCE;
		
		counterVoro++;
		if(counterVoro>3000){
			//ROS_WARN("[PLANNER %d] Voronoi creation failed",__LINE__);
			break;
		}
		
		for(unsigned int k =0;k<currCheck.size();k++){
			int i = currCheck[k].getXCoord();
			int j = currCheck[k].getYCoord();
			currDist = brushCell[i][j];

			if(i > 0){
				if((brushCell[i-1][j] < 0) || (brushCell[i-1][j] > (currDist + 1.0))){
					brushCell[i-1][j] = (currDist + 1);
					(parent[i-1][j]).setCoords(parent[i][j]);
					nextCheck.push_back(PixelCoords(i-1, j));
					stop = false;
				}
				else if(brushCell[i-1][j] == (currDist + 1.0))
				{
					if((parent[i-1][j]).computeSqrDistFrom(parent[i][j]) > parentDistance && brushCell[i-1][j] > MIN_THRESHOLD){
						//~ isOnVoronoi[i-1][j] = true;
						voronoi[i-1][j] = true;
					}
				}
				else if(brushCell[i-1][j] == currDist){
					//~ if((!isOnVoronoi[i-1][j]) && (parent[i-1][j]).computeSqrDistFrom(parent[i][j]) > parentDistance  && brushCell[i][j] > MIN_THRESHOLD){
							//~ isOnVoronoi[i][j] = true;
					if((!voronoi[i-1][j]) && (parent[i-1][j]).computeSqrDistFrom(parent[i][j]) > parentDistance  && brushCell[i][j] > MIN_THRESHOLD){
						voronoi[i][j] = true;
					}
				}
			}
			if(j > 0){
				if((brushCell[i][j-1] < 0) || (brushCell[i][j-1] > (currDist + 1.0))){
					brushCell[i][j-1] = (currDist + 1);
					(parent[i][j-1]).setCoords(parent[i][j]);
					nextCheck.push_back(PixelCoords(i, j-1));
					stop = false;
				}
				else if(brushCell[i][j-1] == (currDist + 1.0)){
					if((parent[i][j-1]).computeSqrDistFrom(parent[i][j]) > parentDistance  && brushCell[i][j-1] > MIN_THRESHOLD){
						//~ isOnVoronoi[i][j-1] = true;
						voronoi[i][j-1] = true;
					}
				}
				else if(brushCell[i][j-1] == currDist){
					//~ if((!isOnVoronoi[i][j-1]) && (parent[i][j-1]).computeSqrDistFrom(parent[i][j]) > parentDistance  && brushCell[i][j] > MIN_THRESHOLD){
						//~ isOnVoronoi[i][j] = true;
					if((!voronoi[i][j-1]) && (parent[i][j-1]).computeSqrDistFrom(parent[i][j]) > parentDistance  && brushCell[i][j] > MIN_THRESHOLD){
						voronoi[i][j] = true;
					}
				}
			}
			//~ if(i < (info.getHeight() - 1)){
			if(i < (mapHeight - 1)){
				
				if((brushCell[i+1][j] < 0) || (brushCell[i+1][j] > (currDist + 1.0))){
					
					brushCell[i+1][j] = (currDist + 1);
					(parent[i+1][j]).setCoords(parent[i][j]);
					nextCheck.push_back(PixelCoords(i+1, j));
					stop = false;
				}
				else if(brushCell[i+1][j] == (currDist + 1.0)){
					
					if((parent[i+1][j]).computeSqrDistFrom(parent[i][j]) > parentDistance && brushCell[i+1][j] > MIN_THRESHOLD){
						//~ isOnVoronoi[i+1][j] = true;
						voronoi[i+1][j] = true;
					}
				}
				else if(brushCell[i+1][j] == currDist){
					
					//~ if((!isOnVoronoi[i+1][j]) && (parent[i+1][j]).computeSqrDistFrom(parent[i][j]) > parentDistance  && brushCell[i][j] > MIN_THRESHOLD){
						//~ isOnVoronoi[i][j] = true;
					if((!voronoi[i+1][j]) && (parent[i+1][j]).computeSqrDistFrom(parent[i][j]) > parentDistance  && brushCell[i][j] > MIN_THRESHOLD){
						voronoi[i][j] = true;
					}
				}
			}
			//~ if(j < (info.getWidth() - 1)){
			if(j < (mapWidth - 1)){
				
				if((brushCell[i][j+1] < 0) || (brushCell[i][j+1] > (currDist + 1.0))){
					brushCell[i][j+1] = (currDist + 1);
					(parent[i][j+1]).setCoords(parent[i][j]);
					nextCheck.push_back(PixelCoords(i, j+1));
					stop = false;
				}
				else if(brushCell[i][j+1] == (currDist + 1.0)){
					if((parent[i][j+1]).computeSqrDistFrom(parent[i][j]) > parentDistance  && brushCell[i][j+1] > MIN_THRESHOLD){
						//~ isOnVoronoi[i][j+1] = true;
						voronoi[i][j+1] = true;
					}
				}
				else if(brushCell[i][j+1] == currDist){
					//~ if((!isOnVoronoi[i][j+1]) && (parent[i][j+1]).computeSqrDistFrom(parent[i][j]) > parentDistance  && brushCell[i][j] > MIN_THRESHOLD){
						//~ isOnVoronoi[i][j] = true;
					if((!voronoi[i][j+1]) && (parent[i][j+1]).computeSqrDistFrom(parent[i][j]) > parentDistance  && brushCell[i][j] > MIN_THRESHOLD){
						voronoi[i][j] = true;
					}
				}
			}
		}

		currCheck.swap(nextCheck);
		nextCheck.clear();
		
		if(currCheck.size()==0)
			return;
	}
}


//~ void Voronoi::trimVoronoiEdges(bool **voronoi, MapInfo &info){
void Voronoi::trimVoronoiEdges(int mapHeight, int mapWidth){
	
	int height, width;
	std::vector<PixelCoords> contaminated, nextContaminated;
	int trimCounter=0;
	
	height = mapHeight;
	width = mapWidth;
	for(int i=0;i<height;i++){
		for(int j=0;j<width;j++){
			if(voronoi[i][j]){
				//~ if(isEdge(voronoi, i, j, info)){
				if( isEdge(i, j, mapHeight, mapWidth) ){
					trimCounter++;
					contaminated.push_back(PixelCoords(i, j));
					voronoi[i][j] = false;
				}
			}
		}
	}
	unsigned int counterVoro=0;
	while(contaminated.size() > 0){
		nextContaminated.clear();
		
		counterVoro++;
		if(counterVoro>3000){
			//ROS_WARN("[PLANNER %d] Voronoi trimming failed",__LINE__);
			break;
		}
		for(unsigned int k=0;k<contaminated.size();k++){
			int i = contaminated[k].getXCoord();
			int j = contaminated[k].getYCoord();
		
			if((i > 0) && voronoi[i-1][j]){
				//~ if(isEdge(voronoi, i-1, j, info)){
				if( isEdge(i-1, j, mapHeight, mapWidth) ){
					nextContaminated.push_back(PixelCoords(i-1, j));
					voronoi[i-1][j] = false;
				}
			}
			else if((j > 0) && voronoi[i][j-1]){
				//~ if(isEdge(voronoi, i, j-1, info)){
				if( isEdge(i, j-1, mapHeight, mapWidth) ){
					nextContaminated.push_back(PixelCoords(i, j-1));
					voronoi[i][j-1] = false;
				}
			}
			else if((i < height) && voronoi[i+1][j]){
				//if(isEdge(voronoi, i+1, j, info)){
				if( isEdge(i+1, j, mapHeight, mapWidth) ){
					nextContaminated.push_back(PixelCoords(i+1, j));
					voronoi[i+1][j] = false;
				}
			}
			else if((j < width) && voronoi[i][j+1]){
				//if(isEdge(voronoi, i, j+1, info)){
				if( isEdge(i, j+1, mapHeight, mapWidth) ){
					nextContaminated.push_back(PixelCoords(i, j+1));
					voronoi[i][j+1] = false;
				}
			}
			else if((i > 0) && (j > 0) && voronoi[i-1][j-1]){
				//if(isEdge(voronoi, i-1, j-1, info)){
				if( isEdge(i-1, j-1, mapHeight, mapWidth) ){
					nextContaminated.push_back(PixelCoords(i-1, j-1));
					voronoi[i-1][j-1] = false;
				}
			}
			else if((i > 0) && (j < width) && voronoi[i-1][j+1]){
				//if(isEdge(voronoi, i-1, j+1, info)){
				if( isEdge(i-1, j+1, mapHeight, mapWidth) ){
					nextContaminated.push_back(PixelCoords(i-1, j+1));
					voronoi[i-1][j+1] = false;
				}
			}
			else if((i < height) && (j > 0) && voronoi[i+1][j-1]){
				//if(isEdge(voronoi, i+1, j-1, info)){
				if( isEdge(i+1, j-1, mapHeight, mapWidth) ){
					nextContaminated.push_back(PixelCoords(i+1, j-1));
					voronoi[i+1][j-1] = false;
				}
			}
			else if((i < height) && (j < width) && voronoi[i+1][j+1]){
				//if(isEdge(voronoi, i+1, j+1, info)){
				if( isEdge(i+1, j+1, mapHeight, mapWidth) ){
					nextContaminated.push_back(PixelCoords(i+1, j+1));
					voronoi[i+1][j+1] = false;
				}
			}
		}
		
		contaminated.swap(nextContaminated);
		nextContaminated.clear();
	}
}


void Voronoi::computeBrushCell(void){
	
	ROS_INFO_NAMED("voronoi","[voronoi_brushcell %d] Computing brushcell",__LINE__);

	//TODO: remove these and use mapAttributes below
	unsigned char **map = mapAttributes->map;
	int mapHeight = mapAttributes->getHeight();
	int mapWidth = mapAttributes->getWidth();
	int xMax = mapAttributes->prevxMax;
	int xMin = mapAttributes->prevxMin;
	int yMax = mapAttributes->prevyMax;
	int yMin = mapAttributes->prevyMin;
	
	bool stop = false;
	int currDist;
	std::vector<PixelCoords> currCheck, nextCheck;

	for(unsigned int i=xMin;i<xMax;i++){
		
		for(unsigned int j=yMin;j<yMax;j++){
			
			//~ isOnVoronoi[i][j] = false;
			//~ voronoi[i][j] = false;
			
			if(map[i][j] < WALL_THRESHOLD) {
				brushCell[i][j] = 0;
				(parent[i][j]).setCoords(i, j);
				currCheck.push_back(PixelCoords(i, j));
			}
			else if(map[i][j] < EMPTY_THRESHOLD){
				brushCell[i][j] = 0;
				(parent[i][j]).setCoords(i, j);
				currCheck.push_back(PixelCoords(i, j));
			}
			else{
				brushCell[i][j] = -1;
				(parent[i][j]).setCoords(-1, -1);
			}
		}
	}
	unsigned int counterVoro=0;
	while(!stop){
		stop = true;
		int parentDistance = VORONOIPARENTDISTANCE;
		
		counterVoro++;
		if(counterVoro>3000){
			//ROS_WARN("[PLANNER %d] Voronoi creation failed",__LINE__);
			break;
		}
		
		for(unsigned int k =0;k<currCheck.size();k++){
			int i = currCheck[k].getXCoord();
			int j = currCheck[k].getYCoord();
			currDist = brushCell[i][j];

			if(i > 0){
				if((brushCell[i-1][j] < 0) || (brushCell[i-1][j] > (currDist + 1.0))){
					brushCell[i-1][j] = (currDist + 1);
					(parent[i-1][j]).setCoords(parent[i][j]);
					nextCheck.push_back(PixelCoords(i-1, j));
					stop = false;
				}
				//else if(brushCell[i-1][j] == (currDist + 1.0))
				//{
					//if((parent[i-1][j]).computeSqrDistFrom(parent[i][j]) > parentDistance && brushCell[i-1][j] > MIN_THRESHOLD){
						////~ isOnVoronoi[i-1][j] = true;
						//voronoi[i-1][j] = true;
					//}
				//}
				//else if(brushCell[i-1][j] == currDist){
					////~ if((!isOnVoronoi[i-1][j]) && (parent[i-1][j]).computeSqrDistFrom(parent[i][j]) > parentDistance  && brushCell[i][j] > MIN_THRESHOLD){
							////~ isOnVoronoi[i][j] = true;
					//if((!voronoi[i-1][j]) && (parent[i-1][j]).computeSqrDistFrom(parent[i][j]) > parentDistance  && brushCell[i][j] > MIN_THRESHOLD){
						//voronoi[i][j] = true;
					//}
				//}
			}
			if(j > 0){
				if((brushCell[i][j-1] < 0) || (brushCell[i][j-1] > (currDist + 1.0))){
					brushCell[i][j-1] = (currDist + 1);
					(parent[i][j-1]).setCoords(parent[i][j]);
					nextCheck.push_back(PixelCoords(i, j-1));
					stop = false;
				}
				//else if(brushCell[i][j-1] == (currDist + 1.0)){
					//if((parent[i][j-1]).computeSqrDistFrom(parent[i][j]) > parentDistance  && brushCell[i][j-1] > MIN_THRESHOLD){
						////~ isOnVoronoi[i][j-1] = true;
						//voronoi[i][j-1] = true;
					//}
				//}
				//else if(brushCell[i][j-1] == currDist){
					////~ if((!isOnVoronoi[i][j-1]) && (parent[i][j-1]).computeSqrDistFrom(parent[i][j]) > parentDistance  && brushCell[i][j] > MIN_THRESHOLD){
						////~ isOnVoronoi[i][j] = true;
					//if((!voronoi[i][j-1]) && (parent[i][j-1]).computeSqrDistFrom(parent[i][j]) > parentDistance  && brushCell[i][j] > MIN_THRESHOLD){
						//voronoi[i][j] = true;
					//}
				//}
			}
			//~ if(i < (info.getHeight() - 1)){
			if(i < (mapHeight - 1)){
				
				if((brushCell[i+1][j] < 0) || (brushCell[i+1][j] > (currDist + 1.0))){
					
					brushCell[i+1][j] = (currDist + 1);
					(parent[i+1][j]).setCoords(parent[i][j]);
					nextCheck.push_back(PixelCoords(i+1, j));
					stop = false;
				}
				//else if(brushCell[i+1][j] == (currDist + 1.0)){
					
					//if((parent[i+1][j]).computeSqrDistFrom(parent[i][j]) > parentDistance && brushCell[i+1][j] > MIN_THRESHOLD){
						////~ isOnVoronoi[i+1][j] = true;
						//voronoi[i+1][j] = true;
					//}
				//}
				//else if(brushCell[i+1][j] == currDist){
					
					////~ if((!isOnVoronoi[i+1][j]) && (parent[i+1][j]).computeSqrDistFrom(parent[i][j]) > parentDistance  && brushCell[i][j] > MIN_THRESHOLD){
						////~ isOnVoronoi[i][j] = true;
					//if((!voronoi[i+1][j]) && (parent[i+1][j]).computeSqrDistFrom(parent[i][j]) > parentDistance  && brushCell[i][j] > MIN_THRESHOLD){
						//voronoi[i][j] = true;
					//}
				//}
			}
			//~ if(j < (info.getWidth() - 1)){
			if(j < (mapWidth - 1)){
				
				if((brushCell[i][j+1] < 0) || (brushCell[i][j+1] > (currDist + 1.0))){
					brushCell[i][j+1] = (currDist + 1);
					(parent[i][j+1]).setCoords(parent[i][j]);
					nextCheck.push_back(PixelCoords(i, j+1));
					stop = false;
				}
				//else if(brushCell[i][j+1] == (currDist + 1.0)){
					//if((parent[i][j+1]).computeSqrDistFrom(parent[i][j]) > parentDistance  && brushCell[i][j+1] > MIN_THRESHOLD){
						////~ isOnVoronoi[i][j+1] = true;
						//voronoi[i][j+1] = true;
					//}
				//}
				//else if(brushCell[i][j+1] == currDist){
					////~ if((!isOnVoronoi[i][j+1]) && (parent[i][j+1]).computeSqrDistFrom(parent[i][j]) > parentDistance  && brushCell[i][j] > MIN_THRESHOLD){
						////~ isOnVoronoi[i][j] = true;
					//if((!voronoi[i][j+1]) && (parent[i][j+1]).computeSqrDistFrom(parent[i][j]) > parentDistance  && brushCell[i][j] > MIN_THRESHOLD){
						//voronoi[i][j] = true;
					//}
				//}
			}
		}

		currCheck.swap(nextCheck);
		nextCheck.clear();
		
		if(currCheck.size()==0)
			return;
	}
}


bool Voronoi::performThinningIteration(unsigned char **img,unsigned char **img2,int width,int height){
	bool changed=false;
	
	//	Four 
	for(int i=1;i<(width-1);i++){
		for(int j=1;j<(height-1);j++){
			//	- 0 0
			// 	1 1 0
			//	- 1	-
			if(					 	   img[i][j-1]==0 	&& img[i+1][j-1]==0 && 
				img[i-1][j]==1 		&& img[i][j]==1 	&& img[i+1][j]==0 	&&
									   img[i][j+1]==1 		)
			{
				img2[i][j]=0;
				changed=true;
				continue;
			}
		}
	}
	for(int i=1;i<(width-1);i++){
		for(int j=1;j<(height-1);j++){
			//	1 1 1
			// 	- 1 -
			//	0 0 0
			if(	img[i-1][j-1]==1 	&& img[i][j-1]==1 	&& img[i+1][j-1]==1 
									&& img[i][j]==1 	&&
				img[i-1][j+1]==0 	&& img[i][j+1]==0 	&& img[i+1][j+1]==0)
			{
				img2[i][j]=0;
				changed=true;
				continue;
			}
		}
	}
	
	if(changed)
		copyElements(img,img2,width,height);
	
	for(int i=1;i<(width-1);i++){
		for(int j=1;j<(height-1);j++){
			//	- 1 -
			// 	1 1 0
			//	- 0 0
			if(						   img[i][j-1]==1 	                    && 
				img[i-1][j]==1 		&& img[i][j]==1 	&& img[i+1][j]==0 	&&
				                       img[i][j+1]==0 	&& img[i+1][j+1]==0)
			{
				img2[i][j]=0;
				changed=true;
				continue;
			}
		}
	}
	for(int i=1;i<(width-1);i++){
		for(int j=1;j<(height-1);j++){
			//	0 0 0
			// 	- 1 -
			//	1 1 1			
			if(	img[i-1][j-1]==0 	&& img[i][j-1]==0 	&& img[i+1][j-1]==0 
									&& img[i][j]==1 	&&
				img[i-1][j+1]==1 	&& img[i][j+1]==1 	&& img[i+1][j+1]==1)
			{
				img2[i][j]=0;
				changed=true;
				continue;
			}
		}
	}
		
	if(changed)
		copyElements(img,img2,width,height);
			
	for(int i=1;i<(width-1);i++){
		for(int j=1;j<(height-1);j++){
			//	- 1 -
			// 	0 1 1
			//	0 0 -
			if(						   img[i][j-1]==1 	&&
				img[i-1][j]==0 		&& img[i][j]==1 	&& img[i+1][j]==1 	&&
				img[i-1][j+1]==0 	&& img[i][j+1]==0 	)
			{
				img2[i][j]=0;
				changed=true;
				continue;
			}
		}
	}
	for(int i=1;i<(width-1);i++){
		for(int j=1;j<(height-1);j++){
			//	1 - 0
			// 	1 1 0
			//	1 - 0	
			if(	img[i-1][j-1]==1 	&& 					   img[i+1][j-1]==0 && 
				img[i-1][j]==1 		&& img[i][j]==1 	&& img[i+1][j]==0 	&&
				img[i-1][j+1]==1 	&& 					   img[i+1][j+1]==0)
			{
				img2[i][j]=0;
				changed=true;
				continue;
			}
		}
	}
	
	if(changed)
		copyElements(img,img2,width,height);
			
	for(int i=1;i<(width-1);i++){
		for(int j=1;j<(height-1);j++){
			//	0 0 -
			// 	0 1 1
			//	- 1 -
			if(	img[i-1][j-1]==0 	&& img[i][j-1]==0 						&& 
				img[i-1][j]==0 		&& img[i][j]==1 	&& img[i+1][j]==1 	
									&& img[i][j+1]==1	)
			{
				img2[i][j]=0;
				changed=true;
				continue;
			}
		}
	}
	for(int i=1;i<(width-1);i++){
		for(int j=1;j<(height-1);j++){
			//	0 - 1
			// 	0 1 1
			//	0 - 1	
			if(	img[i-1][j-1]==0 	&& 					   img[i+1][j-1]==1 && 
				img[i-1][j]==0 		&& img[i][j]==1 	&& img[i+1][j]==1 	&&
				img[i-1][j+1]==0 	&& 					   img[i+1][j+1]==1)
			{
				img2[i][j]=0;
				changed=true;
				continue;
			}
		}
	}

	if(changed)
		copyElements(img,img2,width,height);

	return changed;
}

bool Voronoi::performPruningIteration(unsigned char **img,unsigned char **img2,int width,int height){

	for(int i=1;i<(width-1);i++){
		for(int j=1;j<(height-1);j++){
			
			//	0 0 0
			// 	0 1 0
			//	0 - -
			if(	img[i-1][j-1]==1 		&& img[i][j-1]==0 	&& img[i+1][j-1]==0 && 
				img[i-1][j]==0 			&& img[i][j]==1 	&& img[i+1][j]==0 	&&
				img[i-1][j+1]==0 		&& img[i][j+1]==0	&& img[i+1][j+1]==0		)
			{
				img2[i][j]=0;
				continue;
			}
			//	0 0 0
			// 	- 1 0
			//	- 0 0
			if(	img[i-1][j-1]==0 		&& img[i][j-1]==1 	&& img[i+1][j-1]==0 && 
				img[i-1][j]==0 			&& img[i][j]==1 	&& img[i+1][j]==0 	&&
				img[i-1][j+1]==0 		&& img[i][j+1]==0 	&& img[i+1][j+1]==0 		)
			{
				img2[i][j]=0;continue;
			}
			//	- - 0
			// 	0 1 0
			//	0 0 0 			
			if(	img[i-1][j-1]==0 		&& img[i][j-1]==0 	&& img[i+1][j-1]==1 && 
				img[i-1][j]==0 			&& img[i][j]==1 	&& img[i+1][j]==0 	&&
				img[i-1][j+1]==0 		&& img[i][j+1]==0 	&& img[i+1][j+1]==0 		)
			{
				img2[i][j]=0;continue;
			}
			//	0 0 -
			// 	0 1 -
			//	0 0 0
			if(	img[i-1][j-1]==0 		&& img[i][j-1]==0 	&& img[i+1][j-1]==0 && 
				img[i-1][j]==0 			&& img[i][j]==1 	&& img[i+1][j]==1 	&&
				img[i-1][j+1]==0 		&& img[i][j+1]==0 	&& img[i+1][j+1]==0 		)
			{
				img2[i][j]=0;continue;
			}
			//	0 0 0
			// 	0 1 0
			//	- - 0	
			if(	img[i-1][j-1]==0 		&& img[i][j-1]==0 	&& img[i+1][j-1]==0 && 
				img[i-1][j]==0 			&& img[i][j]==1 	&& img[i+1][j]==0 	&&
				img[i-1][j+1]==0 		&& img[i][j+1]==0 	&& img[i+1][j+1]==1 		)
			{
				img2[i][j]=0;continue;
			}
			//	- 0 0
			// 	- 1 0
			//	0 0 0	
			if(	img[i-1][j-1]==0	&& img[i][j-1]==0 	&& img[i+1][j-1]==0 && 
				img[i-1][j]==0		&& img[i][j]==1 	&& img[i+1][j]==0 	&&
				img[i-1][j+1]==0 		&& img[i][j+1]==1 	&& img[i+1][j+1]==0 		)
			{
				img2[i][j]=0;continue;
			}
			//	0 - -
			// 	0 1 0
			//	0 0 0
			if(	img[i-1][j-1]==0 		&& img[i][j-1]==0 	&& img[i+1][j-1]==0 && 
				img[i-1][j]==0 			&& img[i][j]==1 	&& img[i+1][j]==0 	&&
				img[i-1][j+1]==1 		&& img[i][j+1]==0 	&& img[i+1][j+1]==0 		)
			{
				img2[i][j]=0;continue;
			}
			//	0 0 0
			// 	0 1 -
			//	0 0 -
			if(	img[i-1][j-1]==0 		&& img[i][j-1]==0 	&& img[i+1][j-1]==0 && 
				img[i-1][j]==1 			&& img[i][j]==1 	&& img[i+1][j]==0 	&&
				img[i-1][j+1]==0 		&& img[i][j+1]==0 	&& img[i+1][j+1]==0 		)
			{
				img2[i][j]=0;continue;

			}
			if(	img[i-1][j-1]==0 		&& img[i][j-1]==0 	&& img[i+1][j-1]==0 && 
				img[i-1][j]==0 			&& img[i][j]==1 	&& img[i+1][j]==0 	&&
				img[i-1][j+1]==0 		&& img[i][j+1]==0 	&& img[i+1][j+1]==0 		)
			{
				img2[i][j]=0;continue;

			}
		}
	}

	copyElements(img,img2,width,height);

	return true;
}



bool Voronoi::performFinalPruningIteration(unsigned char **img,unsigned char **img2,int width,int height){

	for(int i=1;i<(width-1);i++){
		for(int j=1;j<(height-1);j++){
			

			if(	img[i-1][j-1]==1 		&& img[i][j-1]==1 	&& img[i+1][j-1]==0 && 
				img[i-1][j]==0 			&& img[i][j]==1 	&& img[i+1][j]==0 	&&
				img[i-1][j+1]==0 		&& img[i][j+1]==0	&& img[i+1][j+1]==0		)
			{
				img2[i][j]=0;continue;
			}

			if(	img[i-1][j-1]==0 		&& img[i][j-1]==0 	&& img[i+1][j-1]==1 && 
				img[i-1][j]==0 			&& img[i][j]==1 	&& img[i+1][j]==1 	&&
				img[i-1][j+1]==0 		&& img[i][j+1]==0 	&& img[i+1][j+1]==0 		)
			{
				img2[i][j]=0;continue;
			}
		
			if(	img[i-1][j-1]==0 		&& img[i][j-1]==0 	&& img[i+1][j-1]==0 && 
				img[i-1][j]==0 			&& img[i][j]==1 	&& img[i+1][j]==0 	&&
				img[i-1][j+1]==0 		&& img[i][j+1]==1 	&& img[i+1][j+1]==1 		)
			{
				img2[i][j]=0;continue;
			}

			if(	img[i-1][j-1]==0 		&& img[i][j-1]==0 	&& img[i+1][j-1]==0 && 
				img[i-1][j]==1 			&& img[i][j]==1 	&& img[i+1][j]==0 	&&
				img[i-1][j+1]==1 		&& img[i][j+1]==0 	&& img[i+1][j+1]==0 		)
			{
				img2[i][j]=0;continue;
			}

			if(	img[i-1][j-1]==0 		&& img[i][j-1]==1 	&& img[i+1][j-1]==1 && 
				img[i-1][j]==0 			&& img[i][j]==1 	&& img[i+1][j]==0 	&&
				img[i-1][j+1]==0 		&& img[i][j+1]==0 	&& img[i+1][j+1]==0 		)
			{
				img2[i][j]=0;continue;
			}

			if(	img[i-1][j-1]==0	&& img[i][j-1]==0 	&& img[i+1][j-1]==0 && 
				img[i-1][j]==0		&& img[i][j]==1 	&& img[i+1][j]==1 	&&
				img[i-1][j+1]==0 		&& img[i][j+1]==0 	&& img[i+1][j+1]==1 		)
			{
				img2[i][j]=0;continue;
			}

			if(	img[i-1][j-1]==0 		&& img[i][j-1]==0 	&& img[i+1][j-1]==0 && 
				img[i-1][j]==0 			&& img[i][j]==1 	&& img[i+1][j]==0 	&&
				img[i-1][j+1]==1 		&& img[i][j+1]==1 	&& img[i+1][j+1]==0 		)
			{
				img2[i][j]=0;continue;
			}

			if(	img[i-1][j-1]==1 		&& img[i][j-1]==0 	&& img[i+1][j-1]==0 && 
				img[i-1][j]==1 			&& img[i][j]==1 	&& img[i+1][j]==0 	&&
				img[i-1][j+1]==0 		&& img[i][j+1]==0 	&& img[i+1][j+1]==0 		)
			{
				img2[i][j]=0;continue;

			}
		}
	}

	copyElements(img,img2,width,height);

	return true;
}


void Voronoi::copyElements(unsigned char **img,unsigned char **img2,int width,int height){
	for(int i=0;i<width;i++)
		for(int j=0;j<height;j++)
			img[i][j]=img2[i][j];
}


//~ bool Voronoi::isEdge(bool **voronoi, int i, int j, MapInfo &info){
bool Voronoi::isEdge(int i, int j, int mapHeight, int mapWidth){
	
	int counter = 0;

	if((i > 0) && voronoi[i-1][j]){
		counter++;
	}
	if((j > 0) && voronoi[i][j-1]){
		counter++;
	}
	//~ if((counter < 2) && (i < info.getHeight()) && voronoi[i+1][j]){
	if((counter < 2) && (i < mapHeight) && voronoi[i+1][j]){
		counter++;
	}
	//~ if((counter < 2) && (j < info.getWidth()) && voronoi[i][j+1]){
	if((counter < 2) && (j < mapWidth) && voronoi[i][j+1]){
		counter++;
	}
	if((counter < 2) && (i > 0) && (j > 0) && voronoi[i-1][j-1]){
		counter++;
	}
	//~ if((counter < 2) && (i > 0) && (j < info.getWidth()) && voronoi[i-1][j+1]){
	if((counter < 2) && (i > 0) && (j < mapWidth) && voronoi[i-1][j+1]){
		counter++;
	}
	//~ if((counter < 2) && (i < info.getHeight()) && (j > 0) && voronoi[i+1][j-1]){
	if((counter < 2) && (i < mapHeight) && (j > 0) && voronoi[i+1][j-1]){
		counter++;
	}
	//~ if((counter < 2) && (i < info.getHeight()) && (j < info.getWidth()) && voronoi[i+1][j+1]){
	if((counter < 2) && (i < mapHeight) && (j < mapWidth) && voronoi[i+1][j+1]){
		counter++;
	}
	
	if(counter < 2)
		return true;
	return false;
}
