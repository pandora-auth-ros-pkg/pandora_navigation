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

#include "pandora_target_selector/closest_unexplored_target_selector.h"

//~ ClosestUnexploredTargetSelector::ClosestUnexploredTargetSelector(void): mapAttributes(MAP_SIZE, MAP_SIZE) {
//~ ClosestUnexploredTargetSelector::ClosestUnexploredTargetSelector(void){
	//~ 
	//~ mapAttributes = new MapAttributes(MAP_SIZE, MAP_SIZE);
	//~ coverage = new Coverage(MAP_SIZE, MAP_SIZE);
//~ 
	//~ 
	//~ field=new int*[MAP_SIZE];
	//~ addField=new int*[MAP_SIZE];
	//~ for(unsigned int i=0;i<MAP_SIZE;i++){
		//~ field[i]=new int[MAP_SIZE];
		//~ addField[i]=new int[MAP_SIZE];
	//~ }
	//~ //TODO: add map and coverage subscribers
	//~ //TODO: initialize map and coverage
//~ }

ClosestUnexploredTargetSelector::ClosestUnexploredTargetSelector(MapAttributes& mapAttr, Coverage& cov): TargetSelector(mapAttr, cov){
//~ ClosestUnexploredTargetSelector::ClosestUnexploredTargetSelector(MapAttributes& mapAttr, Coverage& cov){
	
	//~ mapAttributes = mapAttr;
	//~ coverage = cov;
	
	field=new int*[MAP_SIZE];
	addField=new int*[MAP_SIZE];
	for(unsigned int i=0;i<MAP_SIZE;i++){
		field[i]=new int[MAP_SIZE];
		addField[i]=new int[MAP_SIZE];
	}
	
}




void ClosestUnexploredTargetSelector::selectTarget(PixelCoords* target){
	
	coverageLimits.clear();
	
	ROS_INFO_NAMED("closest_unexplored_target_selector","[closest_unexplored_target_selector %d] Computing coverage limits",__LINE__);
	coverageLimits=findCoverageLimits(mapAttributes.map,mapAttributes.getHeight(), mapAttributes.getWidth(), mapAttributes.robotPose, mapAttributes.prevxMin, mapAttributes.prevxMax, mapAttributes.prevyMin, mapAttributes.prevyMax, field, coverage.coverage, target, addField);
	
	
	
	if( target->getXCoord()==-1  || mapAttributes.map[target->getXCoord()][target->getYCoord()]<=127 ){
		ROS_ERROR_NAMED("closest_unexplored_target_selector","[closest_unexplored_target_selector %d] Can not go with coverage",__LINE__);
		target->setXCoord(-1);
		target->setYCoord(-1);
		return;
	}
	
	ROS_INFO_NAMED("closest_unexplored_target_selector","[closest_unexplored_target_selector %d] Closest unexplored found",__LINE__);
	return;
}



std::vector<PixelCoords> ClosestUnexploredTargetSelector::findCoverageLimits(unsigned char **map,int mapHeight,int mapWidth,Transformation robotPose,int xMin,int xMax,int yMin,int yMax,int **field,unsigned char **coverage,PixelCoords *closest,int **addField){
	bool foundClosest=false;
	float currDist;
	std::vector<PixelCoords> currCheck, nextCheck, coverageLimits;
	
	closest->setXCoord(robotPose.dx+START_X);
	closest->setYCoord(robotPose.dy+START_X);
	
	for(int i=xMin;i<xMax;i++)
	{   for(int j=yMin;j<yMax;j++)
		{   if(!coverage[i][j])
			{   field[i][j] = 0;
			}
			else
			{   field[i][j] = -1;
			}
		}
	}
	field[robotPose.dx+START_X][robotPose.dy+START_Y] = 1;
	currCheck.push_back(PixelCoords(robotPose.dx+START_X,robotPose.dy+START_Y));
	
	unsigned int counter=0;
	do
	{   
		
		for(unsigned int k = 0;k<currCheck.size();k++)
		{   int i = currCheck[k].getXCoord();
			int j = currCheck[k].getYCoord();
			currDist = (float)(field[i][j]);
			
			if(map[i][j]<WALL_THRESHOLD) continue;
			
			int counter=	(coverage[i-1][j-1]>COVERAGE_LIMITS) + \
							(coverage[i][j-1]>COVERAGE_LIMITS) + \
							(coverage[i+1][j-1]>COVERAGE_LIMITS) +\
							(coverage[i-1][j]>COVERAGE_LIMITS) +\
							(coverage[i+1][j]>COVERAGE_LIMITS) +\
							(coverage[i-1][j+1]>COVERAGE_LIMITS) +\
							(coverage[i][j+1]>COVERAGE_LIMITS) + \
							(coverage[i+1][j+1]>COVERAGE_LIMITS) ;

			if(counter!=8){
				bool innercounter=true;
				for(int ii=-1;ii<=1;ii++)								
					for(int jj=-1;jj<=1;jj++)
							innercounter=innercounter&&(map[i+ii][j+jj]>200);
				if(innercounter){
					
					int xmi,xma,ymi,yma;
					
					if((i-CHECK_WALL_TH/2)>=xMin) xmi=i-CHECK_WALL_TH/2;
					else xmi=xMin;
					
					if((j-CHECK_WALL_TH/2)>=yMin) ymi=j-CHECK_WALL_TH/2;
					else ymi=yMin;
					
					if((i+CHECK_WALL_TH/2)<xMax) xma=i+CHECK_WALL_TH/2;
					else xma=xMax-1;
					
					if((j+CHECK_WALL_TH/2)<yMax) yma=j+CHECK_WALL_TH/2;
					else yma=yMax-1;
					
					if(checkPointIfCloseWall(map,mapHeight,mapWidth,PixelCoords(i,j),xmi,xma,ymi,yma,addField)){
						coverageLimits.push_back(PixelCoords(i,j));
						
						if(!foundClosest){
							closest->setXCoord(i);
							closest->setYCoord(j);
							foundClosest=true;
						}	
					}

				}
				continue;
			}

			if(i > 0)
			{   
				if(coverage[i-1][j]){
					if((field[i-1][j] < 0) || (field[i-1][j] > (currDist + 1)))
					{   
						field[i-1][j] = (currDist + 1);
						nextCheck.push_back(PixelCoords(i-1, j));
					}
				}
			}
			if(j > 0)
			{  
				if(coverage[i][j-1]){
					if((field[i][j-1] < 0) || (field[i][j-1] > (currDist + 1)))
					{   
						field[i][j-1] = (currDist + 1);
						nextCheck.push_back(PixelCoords(i, j-1));
					}
				}
			}
			if(i < (mapHeight - 1))
			{   
				if(coverage[i+1][j]){
					if((field[i+1][j] < 0) || (field[i+1][j] > (currDist + 1)))
					{   
						field[i+1][j] = (currDist + 1);
						nextCheck.push_back(PixelCoords(i+1, j));
					}
				}
			}
			if(j < (mapWidth - 1))
			{   
				if(coverage[i][j+1]){
					if((field[i][j+1] < 0) || (field[i][j+1] > (currDist + 1)))
					{   
						field[i][j+1] = (currDist + 1);
						nextCheck.push_back(PixelCoords(i, j+1));
					}
				}
			}
			if(i > 0 && j > 0)
			{   
				if(coverage[i-1][j-1]){
					if((field[i-1][j-1] < 0) || (field[i-1][j-1] > (currDist + 1)))
					{   
						field[i-1][j-1] = (currDist + 1);
						nextCheck.push_back(PixelCoords(i-1, j-1));
					}
				}
			}
			if(i > 0 && j < mapWidth)
			{   
				if(coverage[i-1][j+1]){
					if((field[i-1][j+1] < 0) || (field[i-1][j+1] > (currDist + 1)))
					{   
						field[i-1][j+1] = (currDist + 1);
						nextCheck.push_back(PixelCoords(i-1, j+1));
					}
				}
			}
			if(i < mapHeight && j > 0)
			{   
				if(coverage[i+1][j-1]){
					if((field[i+1][j-1] < 0) || (field[i+1][j-1] > (currDist + 1)))
					{   
						field[i+1][j-1] = (currDist + 1);
						nextCheck.push_back(PixelCoords(i+1, j-1));
					}
				}
			}
			if(i < mapHeight && j < mapWidth)
				{   
					if(coverage[i+1][j+1]){
					if((field[i+1][j+1] < 0) || (field[i+1][j+1] > (currDist + 1)))
					{   
						field[i+1][j+1] = (currDist + 1);
						nextCheck.push_back(PixelCoords(i+1, j+1));
					}
				}
			}
		}
		currCheck.swap(nextCheck);
		nextCheck.clear();
		counter++;
	}while(currCheck.size() > 0);
	return coverageLimits;	
}




bool ClosestUnexploredTargetSelector::checkPointIfCloseWall(unsigned char **map,int mapHeight, int mapWidth,PixelCoords point,int xMin,int xMax,int yMin,int yMax,int **field){
	
	float currDist;
	std::vector<PixelCoords> currCheck, nextCheck;

	for(int i=xMin;i<xMax;i++)
	{   for(int j=yMin;j<yMax;j++)
		{      field[i][j] = -1;
		}
	}
	field[point.getXCoord()][point.getYCoord()] = 1;
	currCheck.push_back(point);
	
	unsigned int counter=0;
	do
	{   
		for(unsigned int k = 0;k<currCheck.size();k++)
		{   int i = currCheck[k].getXCoord();
			int j = currCheck[k].getYCoord();
			currDist = (float)(field[i][j]);

			if(map[i][j]<MAP_LIMITS) return false;
			if(i>=xMax || i<=xMin || j>=yMax || j<=yMin) continue;

			if(i > 0)
			{   
					if((field[i-1][j] < 0) || (field[i-1][j] > (currDist + 1)))
					{   
						field[i-1][j] = (currDist + 1);
						nextCheck.push_back(PixelCoords(i-1, j));
					}
			}
			if(j > 0)
			{  
					if((field[i][j-1] < 0) || (field[i][j-1] > (currDist + 1)))
					{   
						field[i][j-1] = (currDist + 1);
						nextCheck.push_back(PixelCoords(i, j-1));
					}
			}
			if(i < (mapHeight - 1))
			{   
					if((field[i+1][j] < 0) || (field[i+1][j] > (currDist + 1)))
					{   
						field[i+1][j] = (currDist + 1);
						nextCheck.push_back(PixelCoords(i+1, j));
				}
			}
			if(j < (mapWidth - 1))
			{   
					if((field[i][j+1] < 0) || (field[i][j+1] > (currDist + 1)))
					{   
						field[i][j+1] = (currDist + 1);
						nextCheck.push_back(PixelCoords(i, j+1));
					}
			}
			if(i > 0 && j > 0 )
			{   
					if((field[i-1][j-1] < 0) || (field[i-1][j-1] > (currDist + 1)))
					{   
						field[i-1][j-1] = (currDist + 1);
						nextCheck.push_back(PixelCoords(i-1, j-1));
					}
			}
			if(i > 0 && j < mapWidth)
			{   
					if((field[i-1][j+1] < 0) || (field[i-1][j+1] > (currDist + 1)))
					{   
						field[i-1][j+1] = (currDist + 1);
						nextCheck.push_back(PixelCoords(i-1, j+1));
					}
			}
			if(i < mapHeight && j > 0 )
			{   
					if((field[i+1][j-1] < 0) || (field[i+1][j-1] > (currDist + 1)))
					{   
						field[i+1][j-1] = (currDist + 1);
						nextCheck.push_back(PixelCoords(i+1, j-1));
					}
			}
			if(i < mapHeight && j < mapWidth)
			{   

					if((field[i+1][j+1] < 0) || (field[i+1][j+1] > (currDist + 1)))
					{   
						field[i+1][j+1] = (currDist + 1);
						nextCheck.push_back(PixelCoords(i+1, j+1));
					}

			}
		}
		currCheck.swap(nextCheck);
		nextCheck.clear();
		counter++;
	}while(currCheck.size() > 0);
	return true;	
}



