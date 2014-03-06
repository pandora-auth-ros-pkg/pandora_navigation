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

#include "pandora_path_planner/path_generator/dijkstra.h"


bool Dijkstra::checkIfNodesAreConnected(NodesVector &nodes,int id_a,int id_b){
	std::set <int> checked;
	std::vector <int> nextCheck;
	std::vector <int> currCheck;
	currCheck.push_back(id_a);
	while(currCheck.size()!=0){
		for(unsigned int i=0;i<currCheck.size();i++){
			for(unsigned int j=0;j<nodes.nodes[currCheck[i]].neigh.size();j++){
				if(nodes.nodes[currCheck[i]].neighID[j]==(unsigned int)id_b)
					return true;
				else{
					if(checked.count(nodes.nodes[currCheck[i]].neighID[j])==0){
						checked.insert(nodes.nodes[currCheck[i]].neighID[j]);
						nextCheck.push_back(nodes.nodes[currCheck[i]].neighID[j]);
					}
				}
			}
		}
		currCheck.clear();
		currCheck.swap(nextCheck);
	}
	return false;
}

void Dijkstra::findShortestPath(Node startPos,Node targetPos,NodesVector &graphNodes,std::vector<PixelCoords> &shortestPath){
	
	float minDist;
	bool found=false;
	
	int checkNode;
	int tempNode=0;
	int timing=0;
	std::vector<int>tempIds;
	std::vector<float>distance;
	std::vector<PixelCoords>temp;
	
	shortestPath.clear();
	temp.clear();
	
	int startNode=startPos.ID;
	int targetNode=targetPos.ID;
	
	if(!checkIfNodesAreConnected(graphNodes,startNode,targetNode)) return;

	for(unsigned int i=0;i<graphNodes.nodes.size();i++){
		tempIds.push_back(graphNodes.nodes[i].ID);
		distance.push_back(INFINITY_DISTANCE);
		graphNodes.nodes[i].visited=false;
	}
	
	
	graphNodes.nodes[startNode].visited=true;
	checkNode=startNode;
	distance[startNode]=0;
	
	while(tempIds.size()!=0 && !found && timing<5000){	
		minDist=INFINITY_DISTANCE;
		for(unsigned int i=0;i<graphNodes.nodes[checkNode].neigh.size();i++){
			if((int)graphNodes.nodes[checkNode].neighID[i]==targetNode){
				for(unsigned int j=0;j<graphNodes.nodes[targetNode].neigh.size();j++){
					if(!graphNodes.nodes[graphNodes.nodes[targetNode].neighID[j]].visited){
						found=false;
						break;
					}
					found=true;
				}
			}
			if(!graphNodes.nodes[graphNodes.nodes[checkNode].neighID[i]].visited){
				if(distance[graphNodes.nodes[checkNode].neighID[i]]>(distance[checkNode]+graphNodes.nodes[checkNode].dist[i]))
					distance[graphNodes.nodes[checkNode].neighID[i]]=distance[checkNode]+graphNodes.nodes[checkNode].dist[i];
			}
		}
		
		for(unsigned int l=0;l<distance.size();l++){
			if(!graphNodes.nodes[l].visited){
				if(distance[l]<minDist){
					minDist=distance[l];
					tempNode=graphNodes.nodes[l].ID;
				}
			}
		}
		
		for(std::vector<int>::iterator p=tempIds.begin();p!=tempIds.end();p++){
			if(*p==(int)checkNode){
				tempIds.erase(p);
				break;
			}
		}
		checkNode=tempNode;
		graphNodes.nodes[checkNode].visited=true;
		timing++;
	}
	if(timing>=5000)
		return;
	checkNode=targetNode;
	timing=0;
	timing=0;
	while(checkNode!=startNode){
		float minDistance=INFINITY_DISTANCE;
		int minID=0;
		for(unsigned int i=0;i<graphNodes.nodes[checkNode].neigh.size();i++){
			float tempDist=fabs(distance[checkNode]-(distance[graphNodes.nodes[checkNode].neighID[i]]+graphNodes.nodes[checkNode].dist[i]));
			if(tempDist<minDistance){
				minDistance=tempDist;
				minID=graphNodes.nodes[checkNode].neighID[i];
			}
		}
		timing++;
		if(timing>5000){
			shortestPath.clear();
			return;
		}
		shortestPath.push_back(graphNodes.nodes[minID].p);
		checkNode=minID;
		timing++;
		if(timing>=5000){
			shortestPath.clear();
			return;
		}
	}
	for(int i=shortestPath.size()-1;i>=0;i--){
		temp.push_back(shortestPath[i]);
	}
	
	shortestPath.swap(temp);
}


void Dijkstra::findShortestPathAStar(Node startPos,Node targetPos,NodesVector &graphNodes,std::vector<PixelCoords> &shortestPath){
	
	shortestPath.clear();

	int startNode=startPos.ID;
	int targetNode=targetPos.ID;
	if(!checkIfNodesAreConnected(graphNodes,startNode,targetNode))
		return;
	
	std::map<int,float> frontier,visited;
	for(unsigned int i=0;i<graphNodes.nodes.size();i++){
		graphNodes.nodes[i].visited=false;
		graphNodes.nodes[i].parent=-1;
	}
	
	frontier.insert(std::pair<int,float>(startPos.ID,0));
	//~ float dd=pathBrushfire[startPos.p.getXCoord()][startPos.p.getYCoord()]+coverage[startPos.p.getXCoord()][startPos.p.getYCoord()]*0.1;
	//~ frontier.insert(std::pair<int,float>(startPos.ID,dd));
	
	bool targetIsVisited=false;
	while(!targetIsVisited){
		
		//	Find the minimum cost from frontier
		unsigned int idWithMinimumCost=0;
		float minCost=10000000000000000.0;
		for(std::map<int,float>::const_iterator it=frontier.begin();it!=frontier.end();it++){
			
			if( (it->second+graphNodes.nodes[it->first].p.computeDistanceFrom(targetPos.p)) < minCost){
				minCost=it->second+graphNodes.nodes[it->first].p.computeDistanceFrom(targetPos.p);
				idWithMinimumCost=it->first;
			}
		}
		
		//	Expand the specific node
		if(idWithMinimumCost==targetPos.ID){
			targetPos.visited=true;
			targetPos.parent=idWithMinimumCost;
			targetIsVisited=true;
			continue;
		}
		visited.insert(std::pair<int,float>(idWithMinimumCost,frontier[idWithMinimumCost]));
		graphNodes.nodes[idWithMinimumCost].visited=true;
		frontier.erase(idWithMinimumCost);

		for(unsigned int i=0;i<graphNodes.nodes[idWithMinimumCost].neigh.size();i++){
			int neighId=graphNodes.nodes[idWithMinimumCost].neighID[i];
			//	If in visited
			if(visited.find(neighId)!=visited.end())
				continue;
				
			float newDistance=visited[idWithMinimumCost]+graphNodes.nodes[idWithMinimumCost].dist[i];
			
			//------------------------------------------------------------------------------------//
			//~ newDistance+=pathBrushfire[graphNodes.nodes[idWithMinimumCost].neigh[i].getXCoord()][graphNodes.nodes[idWithMinimumCost].neigh[i].getYCoord()]+coverage[graphNodes.nodes[idWithMinimumCost].neigh[i].getXCoord()][graphNodes.nodes[idWithMinimumCost].neigh[i].getYCoord()]*0.1;
			//------------------------------------------------------------------------------------//
			
			if(frontier.find(neighId)==frontier.end()){
				frontier.insert(std::pair<int,float>(neighId,newDistance));
				graphNodes.nodes[neighId].parent=idWithMinimumCost;
			}
			else{
				if(frontier[neighId]>newDistance){
					graphNodes.nodes[neighId].parent=idWithMinimumCost;
					frontier[neighId]=newDistance;
				}
			}
		}
		if(frontier.size()==0)
			break;
	}
	if(frontier.size()==0){
		return;
	}
	std::vector<PixelCoords> ret,t;
	ret.push_back(targetPos.p);
	int parId=targetPos.parent;
	while(parId!=-1){
		ret.push_back(graphNodes.nodes[parId].p);
		parId=graphNodes.nodes[parId].parent;
	}
	ret.push_back(startPos.p);
	for(unsigned int i=0;i<ret.size();i++)
		t.push_back(ret[ret.size()-1-i]);
	
	shortestPath.swap(t);
}


