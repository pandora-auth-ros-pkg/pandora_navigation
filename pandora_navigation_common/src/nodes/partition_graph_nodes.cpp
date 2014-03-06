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

#include "pandora_navigation_common/nodes/partition_graph_nodes.h"

//~ void PartitionGraphNodes::PartitionGraphNodes(void){
//~ PartitionGraphNodes::PartitionGraphNodes(MapAttributes* mapAttr): voronoi(mapAttr,true){
PartitionGraphNodes::PartitionGraphNodes(MapAttributes* mapAttr, Voronoi* vor){
	
	//~ mapAttributes = new MapAttributes;
	mapAttributes = mapAttr;
	voronoi = vor;
	//voronoiGraph = new Voronoi(mapAttr, true);
	
	//~ brushCell=new float*[mapAttr->getHeight()];
	decomp= new int *[mapAttr->getHeight()];
	for(unsigned int i=0;i<mapAttr->getHeight();i++){
		//~ brushCell[i]=new float[mapAttr->getWidth()];
		decomp[i]=new int [mapAttr->getWidth()];
	}
	
	for(unsigned int i=0;i<mapAttr->getHeight();i++)
		for(unsigned int j=0;j<mapAttr->getWidth();j++)
			decomp[i][j]=-1;
			
	uniformPartition.clear();
	targetNeighbors.clear();
	robotNeighbors.clear();
	
	nodes.push_back( Node( PixelCoords(mapAttr->robotPose.dx, mapAttr->robotPose.dy), 0));
	nodes.push_back( Node( PixelCoords(mapAttr->robotPose.dx, mapAttr->robotPose.dy), 1));

	nodes[0].neigh.clear();
	nodes[1].neigh.clear();
	nodes[0].neighID.clear();
	nodes[1].neighID.clear();
	nodes[0].dist.clear();
	nodes[1].dist.clear();
	
}


void PartitionGraphNodes::createIncrementalPartition(PixelCoords startPos, PixelCoords target){
	int id=nodes.size();
	int x=0,y=0;
	
	ROS_INFO_NAMED("partition_graph_nodes","[partition_graph_nodes %d] Computing brushcell",__LINE__);
	
	nodes[0]=(Node(startPos,0));
	nodes[1]=(Node(target,1));
	
	
	ROS_DEBUG_NAMED("partition_graph_nodes_map_values","[Partition Graph %d] Creating Partition  mapAttributes->robotPose.dx=%d mapAttributes->robotPose.dy=%d",__LINE__ , mapAttributes->robotPose.dx, mapAttributes->robotPose.dy);
	ROS_DEBUG_NAMED("partition_graph_nodes_map_values","[Partition Graph %d] Creating Partition  mapAttributes->prevxMax=%d mapAttributes->prevxMin=%d mapAttributes->prevyMax=%d mapAttributes->prevyMin=%d",__LINE__ , mapAttributes->prevxMax,mapAttributes->prevxMin,mapAttributes->prevyMax,mapAttributes->prevyMin);
	
	
	ROS_INFO_NAMED("partition_graph_nodes","[partition_graph_nodes %d] Updating partition graph decomposition",__LINE__);
	performIncrementalPartition( mapAttributes->prevxMax,  mapAttributes->prevxMin, mapAttributes->prevyMax,  mapAttributes->prevyMin,mapAttributes->map,voronoi->brushCell);
	
	ROS_INFO_NAMED("partition_graph_nodes","[partition_graph_nodes %d] Updating graph - nodes' neighbors",__LINE__);
	reinitialiseIncrementalPartition();

	nodes[0]=(Node(startPos,0));
	nodes[1]=(Node(target,1));
	
	for (unsigned int i=0;i<uniformPartition.size();i++ ){
		
		nodes.push_back(Node(uniformPartition[i],id));
		x=uniformPartition[i].getXCoord();
		y=uniformPartition[i].getYCoord();
		decomp[x][y]=id;


		for(int j=-DECOMP_STEP;j<=DECOMP_STEP;j+=DECOMP_STEP){
			for(int k=-DECOMP_STEP;k<=DECOMP_STEP;k+=DECOMP_STEP){
				if(j==0 && k==0){
					continue;
					
				}
				if(decomp[x+j][y+k]>=0){
					nodes[id].makeNeighbor(nodes[decomp[x+j][y+k]]);
				}
				
			}
		}
		id++;
	}	
			
	float dd=nodes[0].p.computeDistanceFrom(nodes[1].p);
			
	if (dd<DECOMP_STEP_DIAG){
		nodes[0].makeNeighbor(nodes[1]); 
		robotNeighbors.push_back(nodes[1].ID);
	}
	for (unsigned int i=2;i<nodes.size();i++){		
		float dd=nodes[0].p.computeDistanceFrom(nodes[i].p);
		if(nodes[i].neigh.size()>0){
			if (dd<DECOMP_TARGET_DIST){
				nodes[0].makeNeighbor(nodes[i]);
				robotNeighbors.push_back(nodes[i].ID);
			}
			
			dd=nodes[1].p.computeDistanceFrom(nodes[i].p);
			if (dd<DECOMP_TARGET_DIST){
				nodes[1].makeNeighbor(nodes[i]);
				targetNeighbors.push_back(nodes[i].ID);
			}			
		}
	}
}		


void PartitionGraphNodes::performIncrementalPartition( int xmax, int xmin,int ymax, int ymin,unsigned char **map,float **brushCell ){
	
	int tempXmin=0,tempYmin=0;
	
	tempXmin=MAP_SIZE/2-((MAP_SIZE/2-xmin)/DECOMP_STEP)*DECOMP_STEP;
	tempYmin=MAP_SIZE/2-((MAP_SIZE/2-ymin)/DECOMP_STEP)*DECOMP_STEP;

	uniformPartition.clear();

	for(int i=tempXmin;i<xmax;i+=DECOMP_STEP){
		for(int j=tempYmin;j<ymax;j+=DECOMP_STEP){
			if (brushCell[i][j]>CLOSE_TO_WALL){
				if(decomp[i][j]==-1){
					uniformPartition.push_back(PixelCoords(i,j));
					decomp[i][j]=-2;
				}
			}
		}
	}
}	


void PartitionGraphNodes::reinitialiseIncrementalPartition(void){
	
	
	nodes[0].neigh.clear();
	nodes[1].neigh.clear();
	nodes[0].neighID.clear();
	nodes[1].neighID.clear();
	nodes[0].dist.clear();
	nodes[1].dist.clear();
	
	for(unsigned int i=0;i<robotNeighbors.size();i++){
		for(unsigned int j=0;j<nodes[robotNeighbors[i]].neighID.size();j++){
			if (nodes[robotNeighbors[i]].neighID[j]==0 ){
				nodes[robotNeighbors[i]].neigh.erase(nodes[robotNeighbors[i]].neigh.begin()+j);
				nodes[robotNeighbors[i]].neighID.erase(nodes[robotNeighbors[i]].neighID.begin()+j);
				nodes[robotNeighbors[i]].dist.erase(nodes[robotNeighbors[i]].dist.begin()+j);	
				
				break;
			}
		}
	}
	
	for(unsigned int i=0;i<targetNeighbors.size();i++){
		for(unsigned int j=0;j<nodes[targetNeighbors[i]].neighID.size();j++){
			if (nodes[targetNeighbors[i]].neighID[j]==1 ){
				nodes[targetNeighbors[i]].neigh.erase(nodes[targetNeighbors[i]].neigh.begin()+j);
				nodes[targetNeighbors[i]].neighID.erase(nodes[targetNeighbors[i]].neighID.begin()+j);	
				nodes[targetNeighbors[i]].dist.erase(nodes[targetNeighbors[i]].dist.begin()+j);
				break;
			}
		}
	}
	
	robotNeighbors.clear();
	targetNeighbors.clear();
	
	for(unsigned int i=0;i<nodes.size();i++){
		int x=0,y=0;
		x=nodes[i].p.getXCoord();
		y=nodes[i].p.getYCoord();
		
		if (voronoi->brushCell[x][y]<CLOSE_TO_WALL){
			nodes[i].neigh.clear();
			nodes[i].neighID.clear();
			nodes[i].dist.clear();
			for(unsigned int j=0;j<nodes.size();j++){
				for(unsigned int k=0;k<nodes[j].neigh.size();k++){
					if((int)nodes[j].neighID[k]==decomp[x][y]){
						nodes[j].neighID.erase(nodes[j].neighID.begin()+k);
						nodes[j].neigh.erase(nodes[j].neigh.begin()+k);
						nodes[j].dist.erase(nodes[j].dist.begin()+k);
						break;
					}
				}
			}
			decomp[x][y]=-1;
		}
	}
}


void PartitionGraphNodes::eliminateNode(int ID,bool keepPosition){
	ROS_INFO_NAMED("partition_graph_nodes","[partition_graph_nodes %d] Eliminating node from partition graph",__LINE__);
	
	int toBeErased=-1;
	for(unsigned int i=0;i<nodes.size();i++){
		if(int (nodes[i].ID)==ID)
			toBeErased=i;
		for(unsigned int j=0;j<nodes[i].neigh.size();j++){
			if(ID==int(nodes[i].neighID[j])){
				nodes[i].neigh.erase(nodes[i].neigh.begin()+j);
				nodes[i].neighID.erase(nodes[i].neighID.begin()+j);
				nodes[i].dist.erase(nodes[i].dist.begin()+j);
				break;
			}
		}
	}
	if(toBeErased!=-1){
		nodes[toBeErased].neigh.clear();
		nodes[toBeErased].neighID.clear();
		nodes[toBeErased].dist.clear();
		if(!keepPosition)
			nodes.erase(nodes.begin()+toBeErased);
	}
}

int PartitionGraphNodes::insertNodeInPartition(Node currNode){
	ROS_INFO_NAMED("partition_graph_nodes","[partition_graph_nodes %d] Inserting node in partition graph",__LINE__);
	
	unsigned int id=nodes.size();
	int x=0, y=0;
	
	x=currNode.p.getXCoord();
	y=currNode.p.getYCoord();
	
	nodes.push_back(Node(currNode.p,id));
	decomp[x][y]=id;
	
	for (int i=(x-DECOMP_STEP_DIAG);i<(x+DECOMP_STEP_DIAG);i++){
		for (int j=(y-DECOMP_STEP_DIAG);j<(y+DECOMP_STEP_DIAG);j++){
			if(i==0 && j==0) continue;
			if (decomp[i][j]>=0){
				nodes[nodes.size()-1].makeNeighbor(nodes[decomp[i][j]]);
			}
		}
	}
	return id;
}


