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

#include "pandora_path_planner/path_generator/voronoi_path_generator/voronoi_path_generator.h"
#include "ros/ros.h"

VoronoiPathGenerator::VoronoiPathGenerator(MapAttributes* mapAttr,Voronoi *v): PathGenerator(mapAttr,v){
	
	mapAttributes = mapAttr;
	
	field = new int*[mapAttr->getHeight()];
	for ( unsigned int i=0; i<mapAttr->getHeight(); i++){
		field[i] = new int[mapAttr->getWidth()];
	}
	
}


bool VoronoiPathGenerator::generatePath( const geometry_msgs::PoseStamped& poseStampedGoal, std::vector<geometry_msgs::PoseStamped>& poseStampedPlan ){
	
	robotPosition = (PixelCoords (mapAttributes->robotPose.dx +START_X, mapAttributes->robotPose.dy+START_Y) );
	pixelGoal = transformPoseStamped2PixelCoords( poseStampedGoal );
	pixelPlan.clear();
	
	
	moveWithVoronoi( robotPosition, pixelGoal, pixelPlan );
	
	
	//TODO: check if pushing final target position is needed
	poseStampedPlan.clear();
	if ( pixelPlan.size()!=0 ){
		
		//minimize path
		ROS_INFO_NAMED("voronoi_path_generator","[voronoi_path_generator %d]: Minimizing path",__LINE__);
		minimizePath( robotPosition, pixelGoal, pixelPlan );
		
		for (unsigned int i=0; i<pixelPlan.size(); i++)
			poseStampedPlan.push_back( transformPixelCoords2PoseStamped(pixelPlan[i]) );
		poseStampedPlan.push_back( transformPixelCoords2PoseStamped(pixelGoal) );
		return true;
	}
	else{
		ROS_INFO_NAMED("voronoi_path_generator","[voronoi_path_generator %d]: Path NOT found",__LINE__);
		return false;
	}
}

void VoronoiPathGenerator::moveWithVoronoi(PixelCoords startPos,PixelCoords target, std::vector<PixelCoords>& wantedPath){
	ROS_INFO_NAMED("voronoi_path_generator","[voronoi_path_generator %d]: Fixing voronoi",__LINE__);
	
	std::vector <PixelCoords> posPath;
	wantedPath.clear();
	//posPath=findOptPathToPoint(field,voronoi,map,info,startPos,target);
	ROS_INFO_NAMED("voronoi_path_generator","[voronoi_path_generator %d]: Calculating Path",__LINE__);
	posPath=findOptPathToPoint(voronoi->voronoi,mapAttributes->map,mapAttributes->getHeight(), mapAttributes->getWidth(),startPos,target);
	if(posPath.size()>DIST){
		unsigned int position=0;
		while(position<posPath.size()){
			wantedPath.push_back(posPath[position]);
			position+=10;
		}
	}
	
}


std::vector<PixelCoords> VoronoiPathGenerator::findOptPathToPoint( bool** voronoi, unsigned char **map,int mapHeight, int mapWidth, PixelCoords position, PixelCoords destination){
	//~ int** field;
	bool finished = false;
	int currX, currY, nextX, nextY;
	float min;
	PixelCoords entryPoint, exitPoint, currPoint, prevPoint;
	std::vector<PixelCoords> pathPoints;


	//~ ROS_INFO("[PLANNER_VORONOI] Going for entry point");
	entryPoint.setCoords(getOptClosest(field,voronoi, map, mapHeight, mapWidth, position));
	//~ ROS_INFO("[PLANNER_VORONOI] Going for exit point");
	exitPoint.setCoords(getOptClosest(field,voronoi, map,mapHeight, mapWidth, destination));

	if((entryPoint.getXCoord() == -1) && (entryPoint.getYCoord() == -1))
	{   std::cout<<"Path drawer: Unreasonable starting position"<<std::endl;
		ROS_ERROR_NAMED("voronoi_path_generator","[voronoi_path_generator %d]: Unreasonable starting position",__LINE__);
		pathPoints.push_back(position);
		return pathPoints;
	}
	else if((exitPoint.getXCoord() == -1) && (exitPoint.getYCoord() == -1))
	{   std::cout<<"Path drawer: Unreachable destination"<<std::endl;
		ROS_ERROR_NAMED("voronoi_path_generator","[voronoi_path_generator %d]: Unreachable destination",__LINE__);
		pathPoints.push_back(position);

		return pathPoints;
	}

	//~ ROS_INFO("[PLANNER_VORONOI] Entry & exit point ok");

	drawOptField(voronoi, field,  mapHeight, mapWidth, exitPoint, entryPoint);


	//~ ROS_INFO("[PLANNER_VORONOI] Draw field ok");

	currPoint.setCoords(entryPoint);
	prevPoint.setCoords(currPoint);
	pathPoints.push_back(position);
	
	long innerCounter=0;
	while(!finished)
	{   nextX = nextY = -1;
		currX = currPoint.getXCoord();
		currY = currPoint.getYCoord();
		pathPoints.push_back(currPoint);
		min = field[currX][currY];

		innerCounter++;
		if(innerCounter>5000)
			break;

		if(currPoint.compareTo(exitPoint))
		{   finished = true;
		}
		else
		{   //Horizontal-vertical search
			if(currX > 0)
			{   if(voronoi[currX-1][currY])
				{   if(field[currX-1][currY] < min)
					{   nextX = currX-1;
						nextY = currY;
						min = field[nextX][nextY];
					}
					else if(field[currX-1][currY] == min && (!prevPoint.compareTo(PixelCoords(currX-1, currY))))
					{   nextX = currX-1;
						nextY = currY;
						min = field[nextX][nextY];
					}
				}
			}
			if(currY > 0)
			{   if(voronoi[currX][currY-1])
				{   if(field[currX][currY-1] < min)
					{   nextX = currX;
						nextY = currY-1;
						min = field[nextX][nextY];
					}
					else if(field[currX][currY-1] == min && (!prevPoint.compareTo(PixelCoords(currX, currY-1))))
					{   nextX = currX;
						nextY = currY-1;
						min = field[nextX][nextY];
					}
				}
			}
			if(currX < mapHeight)
			{   if(voronoi[currX+1][currY])
				{   if(field[currX+1][currY] < min)
					{   nextX = currX+1;
						nextY = currY;
						min = field[nextX][nextY];
					}
					else if(field[currX+1][currY] == min && (!prevPoint.compareTo(PixelCoords(currX+1, currY))))
					{   nextX = currX+1;
						nextY = currY;
						min = field[nextX][nextY];
					}
				}
			}
			if(currY < mapWidth)
			{   if(voronoi[currX][currY+1])
				{   if(field[currX][currY+1] < min)
					{   nextX = currX;
						nextY = currY+1;
						min = field[nextX][nextY];
					}
					else if(field[currX][currY+1] == min && (!prevPoint.compareTo(PixelCoords(currX, currY+1))))
					{   nextX = currX;
						nextY = currY+1;
						min = field[nextX][nextY];
					}
				}
			}
			//Diagonal search
			if(currX > 0 && currY > 0)
			{   if(voronoi[currX-1][currY-1])
				{   if(field[currX-1][currY-1] < min)
					{   nextX = currX-1;
						nextY = currY-1;
						min = field[nextX][nextY];
					}
				}
				else if(field[currX-1][currY-1] == min && (!prevPoint.compareTo(PixelCoords(currX-1, currY-1))))
				{   nextX = currX-1;
					nextY = currY-1;
					min = field[nextX][nextY];
				}
			}
			if(currX < mapHeight && currY > 0)
			{   if(voronoi[currX+1][currY-1])
				{   if(field[currX+1][currY-1] < min)
					{   nextX = currX+1;
						nextY = currY-1;
						min = field[nextX][nextY];
					}
					else if(field[currX+1][currY-1] == min && (!prevPoint.compareTo(PixelCoords(currX+1, currY-1))))
					{   nextX = currX+1;
						nextY = currY-1;
						min = field[nextX][nextY];
					}
				}
			}
			if(currX > 0 && currY < mapWidth)
			{   if(voronoi[currX-1][currY+1])
				{   if(field[currX-1][currY+1] < min)
					{   nextX = currX-1;
						nextY = currY+1;
						min = field[nextX][nextY];
					}
					else if(field[currX-1][currY+1] == min && (!prevPoint.compareTo(PixelCoords(currX-1, currY+1))))
					{   nextX = currX-1;
						nextY = currY+1;
						min = field[nextX][nextY];
					}
				}
			}
			if(currX < mapHeight && currY < mapWidth)
			{   if(voronoi[currX+1][currY+1])
				{   if(field[currX+1][currY+1] < min)
					{   nextX = currX+1;
						nextY = currY+1;
						min = field[nextX][nextY];
					}
					else if(field[currX+1][currY+1] == min && (!prevPoint.compareTo(PixelCoords(currX+1, currY+1))))
					{   nextX = currX+1;
						nextY = currY+1;
						min = field[nextX][nextY];
					}
				}
			}
			if(nextX == -1 || nextY == -1)
			{   
				#ifdef PRINT_FLOW
					cout<<"optPath : 530\n";
				#endif
				if(pathPoints.size() > 1)
				{   //std::cout<<"Path drawer: Unreachable destination"<<std::endl;
					pathPoints.clear();
					pathPoints.push_back(position);
				}
				else
				{   //std::cout<<"Path drawer: Zero length voronoi path calculated"<<std::endl;
					pathPoints.push_back(exitPoint);
				}
				pathPoints.push_back(destination);

				return pathPoints;
			}
			else
			{   prevPoint.setCoords(currPoint);
				currPoint.setCoords(nextX, nextY);
			}
		}
	}
	if(innerCounter<5000)
		pathPoints.push_back(destination);
	else
		pathPoints.clear();
	#ifdef PRINT_FLOW
		cout<<"optPath : done-------------------\n\n";
	#endif
	return pathPoints;
}


void VoronoiPathGenerator::drawOptField(bool **voronoi, int **field, int mapHeight, int mapWidth, PixelCoords position, PixelCoords destination)
{   bool endPoint = false;
	float currDist;
	std::vector<PixelCoords> currCheck, nextCheck;

	for(int i=0;i<mapHeight;i++)
	{   for(int j=0;j<mapWidth;j++)
		{   if(!voronoi[i][j])
			{   field[i][j] = 0;
			}
			else
			{   field[i][j] = -1;
			}
		}
	}
	field[position.getXCoord()][position.getYCoord()] = 1;
	currCheck.push_back(position);

	do
	{   for(unsigned int k = 0;k<currCheck.size();k++)
		{   int i = currCheck[k].getXCoord();
			int j = currCheck[k].getYCoord();
			currDist = (float)(field[i][j]);

			if(i > 0)
			{   if((field[i-1][j] < 0) ||
				   (field[i-1][j] > (currDist + 1)))
				{   field[i-1][j] = (currDist + 1);
					nextCheck.push_back(PixelCoords(i-1, j));
				}
			}
			if(j > 0)
			{   if((field[i][j-1] < 0) ||
				(field[i][j-1] > (currDist + 1)))
				{   field[i][j-1] = (currDist + 1);
					nextCheck.push_back(PixelCoords(i, j-1));
				}
			}
			if(i < (mapHeight - 1))
			{   if((field[i+1][j] < 0) ||
				   (field[i+1][j] > (currDist + 1)))
				{   field[i+1][j] = (currDist + 1);
					nextCheck.push_back(PixelCoords(i+1, j));
				}
			}
			if(j < (mapWidth - 1))
			{   if((field[i][j+1] < 0) ||
				(field[i][j+1] > (currDist + 1)))
				{   field[i][j+1] = (currDist + 1);
					nextCheck.push_back(PixelCoords(i, j+1));
				}
			}
			if(i > 0 && j > 0)
			{   if((field[i-1][j-1] < 0) ||
				(field[i-1][j-1] > (currDist + 2)))
				{   field[i-1][j-1] = (currDist + 2);
					nextCheck.push_back(PixelCoords(i-1, j-1));
				}
			}
			if(i > 0 && j < mapWidth)
			{   if((field[i-1][j+1] < 0) ||
				   (field[i-1][j+1] > (currDist + 2)))
				{   field[i-1][j+1] = (currDist + 2);
					nextCheck.push_back(PixelCoords(i-1, j+1));
				}
			}
			if(i < mapHeight && j > 0)
			{   if((field[i+1][j-1] < 0) ||
				   (field[i+1][j-1] > (currDist + 2)))
				{   field[i+1][j-1] = (currDist + 2);
					nextCheck.push_back(PixelCoords(i+1, j-1));
				}
			}
			if(i < mapHeight && j < mapWidth)
				{   if((field[i+1][j+1] < 0) ||
				   (field[i+1][j+1] > (currDist + 2)))
				{   field[i+1][j+1] = (currDist + 2);
					nextCheck.push_back(PixelCoords(i+1, j+1));
				}
			}
			if(destination.compareTo(PixelCoords(i, j)))
			{   endPoint = true;
			}
		}

		if(endPoint)
			nextCheck.clear();

		currCheck.swap(nextCheck);
		nextCheck.clear();
	}while(currCheck.size() > 0);
}


PixelCoords VoronoiPathGenerator::getOptClosest(int **brushCell,bool **voronoi, unsigned char **map, int mapHeight, int mapWidth, PixelCoords position)
{   bool stop = false;
	int currDist;

	PixelCoords closestPoint;
	std::vector<PixelCoords> currCheck, nextCheck;

	if(map[position.getXCoord()][position.getYCoord()] < EMPTY_THRESHOLD)
	{   return PixelCoords();
	}

	for(int i=0;i<mapHeight;i++)
	{ 
		for(int j=0;j<mapWidth;j++)
		{   if(map[i][j] < EMPTY_THRESHOLD)
				brushCell[i][j] = 0;
			else
				brushCell[i][j] = -1;
		}
	}
	brushCell[position.getXCoord()][position.getYCoord()] = 1;
	currCheck.push_back(position);

	while(!stop)
	{   for(unsigned int k=0;k<currCheck.size();k++)
		{   int i = (currCheck[k]).getXCoord();
			int j = (currCheck[k]).getYCoord();
			currDist = brushCell[i][j];

			if(i > 0)
			{   if(!(voronoi[i-1][j]))
				{   if((brushCell[i-1][j] < 0) ||
					   (brushCell[i-1][j] > (currDist + 1)))
					{   brushCell[i-1][j] = (currDist + 1);
						nextCheck.push_back(PixelCoords(i-1, j));
					}
				}
				else
				{   stop = true;
					closestPoint.setCoords(i-1, j);
				}
			}
			if(j > 0)
			{   if(!(voronoi[i][j-1]))
				{   if((brushCell[i][j-1] < 0) ||
					   (brushCell[i][j-1] > (currDist + 1)))
					{   brushCell[i][j-1] = (currDist + 1);
						nextCheck.push_back(PixelCoords(i, j-1));
					}
				}
				else
				{   stop = true;
					closestPoint.setCoords(i, j-1);
				}
			}
			if(i < (mapHeight - 1))
			{   if(!(voronoi[i+1][j]))
				{   if((brushCell[i+1][j] < 0) ||
					   (brushCell[i+1][j] > (currDist + 1)))
					{   brushCell[i+1][j] = (currDist + 1);
						nextCheck.push_back(PixelCoords(i+1, j));
					}
				}
				else
				{   stop = true;
					closestPoint.setCoords(i+1, j);
				}
			}
			if(j < (mapWidth - 1))
			{   if(!(voronoi[i][j+1]))
				{   if((brushCell[i][j+1] < 0) ||
					   (brushCell[i][j+1] > (currDist + 1)))
					{   brushCell[i][j+1] = (currDist + 1);
						nextCheck.push_back(PixelCoords(i, j+1));
					}
				}
				else
				{   stop = true;
					closestPoint.setCoords(i, j+1);
				}
			}
		}
		currCheck.swap(nextCheck);
		nextCheck.clear();
	}

	return closestPoint;
}


