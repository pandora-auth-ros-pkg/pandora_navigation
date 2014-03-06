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

#include "pandora_path_planner/path_generator/path_generator.h"

PathGenerator::PathGenerator(MapAttributes* mapAttr,Voronoi * v): mapAttributes(mapAttr), shortestPathFinder() {
	mapAttributes =mapAttr;
	voronoi=v;
}

//~ void PathPlanner::minimizePath(PixelCoords startPos,PixelCoords target){
void PathGenerator::minimizePath(PixelCoords startPos,PixelCoords target, std::vector<PixelCoords>& wantedPath){
	std::vector <PixelCoords> posPath=wantedPath;
	std::vector <PixelCoords> bestNodes=wantedPath;
	
	PixelCoords *tempPixels = new PixelCoords [8];
	double pathFitness=calcPathFitness(posPath);
	double fitness=0;
	double checkFitness=0;

	int mutation=32;
	int count=0;
	bool changed;
	std::vector<int> toBerased;

	while(count++<100000){
		changed=false;
		for(unsigned int index=1;index<wantedPath.size();index++){
			int k=0;
			posPath=wantedPath;
			
		
			if(index==1)
				fitness=calcFitness(wantedPath[index],startPos,wantedPath[index+1]);
			else if(index==wantedPath.size()-1)
				fitness=calcFitness(wantedPath[index],target,wantedPath[index-1]);
			else
				fitness=calcFitness(wantedPath[index],wantedPath[index-1],wantedPath[index+1]);
			for(int i=-1;i<2;i++){
				for(int j=-1;j<2;j++){
					if(!(i==0&&j==0)){
						tempPixels[k]=PixelCoords(wantedPath[index].getXCoord()+i*mutation,wantedPath[index].getYCoord()+j*mutation);
						posPath[index]=tempPixels[k];
						if(checkIfValidPathMutation(posPath,index,startPos,target)){
							if(index==0)
								checkFitness=calcFitness(posPath[index],startPos,posPath[index+1]);
							else if(index==posPath.size()-1)
								checkFitness=calcFitness(posPath[index],target,posPath[index-1]);
							else
								checkFitness=calcFitness(posPath[index],posPath[index-1],posPath[index+1]);
							if(checkFitness<fitness){
								fitness=checkFitness;
								bestNodes[index]=posPath[index];
								changed=true;
								
							}
						}
					}
				}
			}
			if(changed){
				wantedPath[index]=bestNodes[index];
			
			}
		}
		if(!changed){
			mutation/=2;
			if(mutation<1)
				break;
		}
	}
	//maybe this is not needed
	pathFitness=calcPathFitness(wantedPath);
}


bool PathGenerator::checkIfValidPathMutation(std::vector<PixelCoords> &path, unsigned int index, PixelCoords startPos, PixelCoords target){
	if(index>0 && index<path.size()-1){
		if(!isLineValid(path[index],path[index+1],mapAttributes->map,voronoi->brushCell,1.2)\
		   ||!isLineValid(path[index],path[index-1],mapAttributes->map,voronoi->brushCell,1.2))
			return false;
	}
	else if(index==0){
		if(!isLineValid(path[index],path[index+1],mapAttributes->map,voronoi->brushCell,1.2)\
		   ||!isLineValid(path[index],startPos,mapAttributes->map,voronoi->brushCell,1.2))
			return false;
	}
	else if(index==path.size()-1){
		if(!isLineValid(path[index],path[index-1],mapAttributes->map,voronoi->brushCell,1.2)\
		   ||!isLineValid(path[index],target,mapAttributes->map,voronoi->brushCell,1.2))
			return false;
	}
	return true;
}


double PathGenerator::calcFitness(PixelCoords currPoint,PixelCoords prevPoint,PixelCoords nextPoint){
	
	return currPoint.computeSqrDistFrom(prevPoint)+currPoint.computeSqrDistFrom(nextPoint);
	
}
 
double PathGenerator::calcPathFitness(std::vector<PixelCoords> &posPath){
	double fitness=0;
	for(unsigned int i=0;i<posPath.size()-1;i++)
		fitness+=posPath[i].computeSqrDistFrom(posPath[i+1]);
	return fitness;
}


bool PathGenerator::isLineValid(PixelCoords p1,PixelCoords p2, unsigned char **map,float **brushCell,float op){
	int x1,x2,y1,y2,temp;
	x1=p1.getXCoord();
	x2=p2.getXCoord();
	y1=p1.getYCoord();
	y2=p2.getYCoord();
	if(x1==x2){
		
		if(y1>y2){
			temp=y1;
			y1=y2;
			y2=temp;
		}
		for(int i=y1;i<y2;i++)
			if(map[x1][i]<130 || brushCell[x1][i]<(WALL_DISTANCE)*op) return false;
		return true;
	}
	if(x1>x2){
		temp=x1;
		x1=x2;
		x2=temp;
		temp=y1;
		y1=y2;
		y2=temp;
	}
	float l=((float)(y2-(float)y1)/(float)(x2-(float)x1));
	if((x2-x1)>abs(y2-y1)){	//Must find y's
		for(int i=x1;i<x2;i++)
			if(map[i][(int(l*i-l*x1+(float)y1))]<130 || brushCell[i][(int(l*i-l*x1+(float)y1))]<(WALL_DISTANCE)*op) return false;
	}
	else{					//Must find x's
		if(y1>y2){
			temp=x1;
			x1=x2;
			x2=temp;
			temp=y1;
			y1=y2;
			y2=temp;
		}
		for(int i=y1;i<y2;i++)
			if(map[int((i-(float)y1+l*x1)/l)][i]<130 || brushCell[int((i-(float)y1+l*x1)/l)][i]<(WALL_DISTANCE)*op) return false;		
	}
	return true;
}


PixelCoords PathGenerator::transformPoseStamped2PixelCoords(geometry_msgs::PoseStamped poseStamped){
	
	int x = poseStamped.pose.position.x;
	int y = poseStamped.pose.position.y;
	
	return PixelCoords(x,y) ;
	
}



geometry_msgs::PoseStamped PathGenerator::transformPixelCoords2PoseStamped(PixelCoords pixelCoords){
	
	float x = pixelCoords.getXCoord();
	float y = pixelCoords.getYCoord();
	
	geometry_msgs::PoseStamped poseStamped;
	
	poseStamped.pose.position.x = x;
	poseStamped.pose.position.y = y;
	
	return poseStamped;
	
	
}

//~ void PathPlanner::resetTargets(void){
	//~ goal.clear();
	//~ nodesQueue.clear();
	//~ currGoal=0;	
	//~ lockTarget=false;
//~ }



//bool PathGenerator::checkIfNodesAreConnected(NodesVector &nodes,int id_a,int id_b){
	//set <int> checked;
	//vector <int> nextCheck;
	//vector <int> currCheck;
	//currCheck.push_back(id_a);
	//while(currCheck.size()!=0){
		//for(unsigned int i=0;i<currCheck.size();i++){
			//for(unsigned int j=0;j<nodes.nodes[currCheck[i]].neigh.size();j++){
				//if(nodes.nodes[currCheck[i]].neighID[j]==(unsigned int)id_b)
					//return true;
				//else{
					//if(checked.count(nodes.nodes[currCheck[i]].neighID[j])==0){
						//checked.insert(nodes.nodes[currCheck[i]].neighID[j]);
						//nextCheck.push_back(nodes.nodes[currCheck[i]].neighID[j]);
					//}
				//}
			//}
		//}
		//currCheck.clear();
		//currCheck.swap(nextCheck);
	//}
	//return false;
//}

//void PathGenerator::findShortestPath(Node startPos,Node targetPos,NodesVector &graphNodes,std::vector<PixelCoords> &shortestPath){
	
	//float minDist;
	//bool found=false;
	
	//int checkNode;
	//int tempNode=0;
	//int timing=0;
	//vector<int>tempIds;
	//vector<float>distance;
	//vector<PixelCoords>temp;
	
	//shortestPath.clear();
	//temp.clear();
	
	//int startNode=startPos.ID;
	//int targetNode=targetPos.ID;
	
	//if(!checkIfNodesAreConnected(graphNodes,startNode,targetNode)) return;

	//for(unsigned int i=0;i<graphNodes.nodes.size();i++){
		//tempIds.push_back(graphNodes.nodes[i].ID);
		//distance.push_back(INFINITY_DISTANCE);
		//graphNodes.nodes[i].visited=false;
	//}
	
	
	//graphNodes.nodes[startNode].visited=true;
	//checkNode=startNode;
	//distance[startNode]=0;
	
	//while(tempIds.size()!=0 && !found && timing<5000){	
		//minDist=INFINITY_DISTANCE;
		//for(unsigned int i=0;i<graphNodes.nodes[checkNode].neigh.size();i++){
			//if((int)graphNodes.nodes[checkNode].neighID[i]==targetNode){
				//for(unsigned int j=0;j<graphNodes.nodes[targetNode].neigh.size();j++){
					//if(!graphNodes.nodes[graphNodes.nodes[targetNode].neighID[j]].visited){
						//found=false;
						//break;
					//}
					//found=true;
				//}
			//}
			//if(!graphNodes.nodes[graphNodes.nodes[checkNode].neighID[i]].visited){
				//if(distance[graphNodes.nodes[checkNode].neighID[i]]>(distance[checkNode]+graphNodes.nodes[checkNode].dist[i]))
					//distance[graphNodes.nodes[checkNode].neighID[i]]=distance[checkNode]+graphNodes.nodes[checkNode].dist[i];
			//}
		//}
		
		//for(unsigned int l=0;l<distance.size();l++){
			//if(!graphNodes.nodes[l].visited){
				//if(distance[l]<minDist){
					//minDist=distance[l];
					//tempNode=graphNodes.nodes[l].ID;
				//}
			//}
		//}
		
		//for(vector<int>::iterator p=tempIds.begin();p!=tempIds.end();p++){
			//if(*p==(int)checkNode){
				//tempIds.erase(p);
				//break;
			//}
		//}
		//checkNode=tempNode;
		//graphNodes.nodes[checkNode].visited=true;
		//timing++;
	//}
	//if(timing>=5000)
		//return;
	//checkNode=targetNode;
	//timing=0;
	//timing=0;
	//while(checkNode!=startNode){
		//float minDistance=INFINITY_DISTANCE;
		//int minID=0;
		//for(unsigned int i=0;i<graphNodes.nodes[checkNode].neigh.size();i++){
			//float tempDist=fabs(distance[checkNode]-(distance[graphNodes.nodes[checkNode].neighID[i]]+graphNodes.nodes[checkNode].dist[i]));
			//if(tempDist<minDistance){
				//minDistance=tempDist;
				//minID=graphNodes.nodes[checkNode].neighID[i];
			//}
		//}
		//timing++;
		//if(timing>5000){
			//shortestPath.clear();
			//return;
		//}
		//shortestPath.push_back(graphNodes.nodes[minID].p);
		//checkNode=minID;
		//timing++;
		//if(timing>=5000){
			//shortestPath.clear();
			//return;
		//}
	//}
	//for(int i=shortestPath.size()-1;i>=0;i--){
		//temp.push_back(shortestPath[i]);
	//}
	
	//shortestPath.swap(temp);
//}
