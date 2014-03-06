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

#include "pandora_navigation_common/nodes/voronoi_nodes.h"
#include "ros/ros.h"

//~ void VoronoiNodes::VoronoiNodes(void){
//void VoronoiNodes::VoronoiNodes(const MapAttributes* mapAttr, const Coverage cov){
VoronoiNodes::VoronoiNodes(MapAttributes* mapAttr, Coverage* cov): voronoi(mapAttr){
	
	nodes.clear();
	
	//~ mapAttributes = new MapAttributes;
	mapAttributes = mapAttr;
	
	//~ coverage = new Coverage;
	coverage = cov;
	
	//voronoi = new Voronoi(mapAttr);
	
	
	//~ field = new int*[MAP_SIZE];
	//~ isNode = new bool*[MAP_SIZE];
	//~ for ( unsigned int i=0; i<MAP_SIZE; i++){
		//~ field[i] = new int[MAP_SIZE];
		//~ isNode[i] = new bool[MAP_SIZE];
	//~ }
	field = new int*[mapAttr->getHeight()];
	isNode = new bool*[mapAttr->getHeight()];
	
	for ( unsigned int i=0; i<mapAttr->getHeight(); i++){
		field[i] = new int[mapAttr->getWidth()];
		isNode[i] = new bool[mapAttr->getWidth()];
	}
	
	
	//~ TODO: maybe move this into function
	//~ for(int i=xMin;i<xMax;i++)
		//~ for(int j=yMin;j<yMax;j++)
			//~ isNode[i][j]=false;
	
	
	//~ createVoronodes();
	
	
}


void VoronoiNodes::assignIDs(void){
	
	for(unsigned int i=0;i<nodes.size();i++)
		nodes[i].ID=i;
		
	for(unsigned int i=0;i<nodes.size();i++){
		nodes[i].neighID.clear();
		for(unsigned int j=0;j<nodes[i].neigh.size();j++){
			if ( (transformPixelCoordsToNode(nodes[i].neigh[j])==NULL )) ROS_ERROR( "transformPixelCoordsToNode(nodes[%d].neigh[%d])==NULL",i,j );
			nodes[i].neighID.push_back((transformPixelCoordsToNode(nodes[i].neigh[j]))->ID);
		}
	}
}


//~ bool Voronodes::checkCrosspathNodes(unsigned int i,unsigned int j,bool **voronoi){
bool VoronoiNodes::checkCrosspathNodes(unsigned int i,unsigned int j){
	//	* - 1
	//	* 1 1
	//	* - -
	if(	 voronoi.voronoi[i+1][j-1] && voronoi.voronoi[i+1][j] && 
		(voronoi.voronoi[i-1][j-1] || voronoi.voronoi[i-1][j] || voronoi.voronoi[i-1][j+1])) return false;
	//	* * *
	//	- 1 -
	//	- 1 1
	if(	 voronoi.voronoi[i][j+1] && voronoi.voronoi[i+1][j+1] && 
		(voronoi.voronoi[i-1][j-1] || voronoi.voronoi[i][j-1] || voronoi.voronoi[i+1][j-1])) return false;
	//	- - *
	//	1 1 *
	//	1 - *
	if(	 voronoi.voronoi[i-1][j] && voronoi.voronoi[i-1][j+1] && 
		(voronoi.voronoi[i+1][j-1] || voronoi.voronoi[i+1][j] || voronoi.voronoi[i+1][j+1])) return false;
	//	1 1 -
	//	- 1 -
	//	* * *
	if(	 voronoi.voronoi[i-1][j-1] && voronoi.voronoi[i][j-1] && 
		(voronoi.voronoi[i-1][j+1] || voronoi.voronoi[i][j+1] || voronoi.voronoi[i+1][j+1])) return false;
		
	//	* - -
	//	* 1 1
	//	* - 1
	if(	 voronoi.voronoi[i+1][j+1] && voronoi.voronoi[i+1][j] && 
		(voronoi.voronoi[i-1][j-1] || voronoi.voronoi[i-1][j] || voronoi.voronoi[i-1][j+1])) return false;
	//	* * *
	//	- 1 -
	//	1 1 -
	if(	 voronoi.voronoi[i][j+1] && voronoi.voronoi[i-1][j+1] && 
		(voronoi.voronoi[i-1][j-1] || voronoi.voronoi[i][j-1] || voronoi.voronoi[i+1][j-1])) return false;
	//	1 - *
	//	1 1 *
	//	- - *
	if(	 voronoi.voronoi[i-1][j] && voronoi.voronoi[i-1][j-1] && 
		(voronoi.voronoi[i+1][j-1] || voronoi.voronoi[i+1][j] || voronoi.voronoi[i+1][j+1])) return false;
	//	- 1 1
	//	- 1 -
	//	* * *
	if(	 voronoi.voronoi[i+1][j-1] && voronoi.voronoi[i][j-1] && 
		(voronoi.voronoi[i-1][j+1] || voronoi.voronoi[i][j+1] || voronoi.voronoi[i+1][j+1])) return false;
	
	return true;
}


bool VoronoiNodes::checkForNeigbor(PixelCoords neigbor,std::vector<PixelCoords> &n){
	for(unsigned int i=0;i<n.size();i++)
		if(neigbor.getXCoord()==n[i].getXCoord() && neigbor.getYCoord()==n[i].getYCoord())
			return true;
	return false;
}


//~ bool checkIfNodeInVoronodes(Voronode *n,Voronodes &nodes){
bool VoronoiNodes::checkIfNodeInVoronodes(Node *n){
	for(unsigned int i=0;i<nodes.size();i++)	
	{
		if(n->p.getXCoord()==nodes[i].p.getXCoord() && n->p.getYCoord()==nodes[i].p.getYCoord())
			return true;
	}
	return false;
}


void VoronoiNodes::createVoronodes(void){
	
	ROS_DEBUG_NAMED("voronoi_nodes_map_values","[voronoi_nodes %d] mapAttributes->robotPose.dx=%d mapAttributes->robotPose.dy=%d",__LINE__ , mapAttributes->robotPose.dx, mapAttributes->robotPose.dy);
	ROS_DEBUG_NAMED("voronoi_nodes_map_values","[voronoi_nodes %d] mapAttributes->prevxMax=%d mapAttributes->prevxMin=%d mapAttributes->prevyMax=%d mapAttributes->prevyMin=%d",__LINE__ , mapAttributes->prevxMax,mapAttributes->prevxMin,mapAttributes->prevyMax,mapAttributes->prevyMin);
	
	
	
	if(mapAttributes->map[mapAttributes->robotPose.dx+START_X][mapAttributes->robotPose.dy+START_Y]<EMPTY_THRESHOLD){
		ROS_ERROR_NAMED("voronoi_nodes","[voronoi_nodes %d] Did not find voronodes",__LINE__);	
		return;
	}
	
	ROS_INFO_NAMED("voronoi_nodes","[voronoi_nodes %d] Fixing voronoi",__LINE__);
	voronoi.fixVoronoi();
	
	ROS_INFO_NAMED("voronoi_nodes","[voronoi_nodes %d] Detecting voronoi nodes",__LINE__);	
	detectNodes(voronoi.voronoi, mapAttributes->getHeight(), mapAttributes->getWidth(),mapAttributes->prevxMin,mapAttributes->prevxMax,mapAttributes->prevyMin,mapAttributes->prevyMax,coverage->coverage,isNode,field,mapAttributes->map,mapAttributes->robotPose);
	
	ROS_INFO_NAMED("voronoi_nodes","[voronoi_nodes %d] Manipulating voronoi nodes",__LINE__);	
	manipulateNodes();
	
	ROS_INFO_NAMED("voronoi_nodes","[voronoi_nodes %d] Voronoi_nodes created",__LINE__);
	
}


void VoronoiNodes::detectNodes(bool **voronoi, int mapHeight, int mapWidth,int xMin,int xMax,int yMin,int yMax, unsigned char **coverage,bool **isNode,int **field,unsigned char ** map,Transformation robotPose){
	
	nodes.clear();
	int counter=0;
	
	for(int i=xMin;i<xMax;i++)
		for(int j=yMin;j<yMax;j++)
			isNode[i][j]=false;
			
	
	for(int i=xMin;i<xMax;i++){
		for(int j=yMin;j<yMax;j++){
			if(voronoi[i][j]){
				//	Check for voronoi edges
				if(	voronoi[i-1][j-1] 		&& !voronoi[i][j-1] 	&& !voronoi[i+1][j-1] && 
					!voronoi[i-1][j] 		&& voronoi[i][j] 		&& !voronoi[i+1][j] 	&&
					!voronoi[i-1][j+1] 		&& !voronoi[i][j+1] 	&& !voronoi[i+1][j+1]  && coverage[i][j]<127)
				{
					nodes.push_back(Node(PixelCoords(i,j)));
					counter++;
					isNode[i][j]=true;
					continue;	
				}
				if(	!voronoi[i-1][j-1] 		&& voronoi[i][j-1] 	&& !voronoi[i+1][j-1] && 
					!voronoi[i-1][j] 		&& voronoi[i][j] 		&& !voronoi[i+1][j] 	&&
					!voronoi[i-1][j+1] 		&& !voronoi[i][j+1] 	&& !voronoi[i+1][j+1]  && coverage[i][j]<127)
				{
					nodes.push_back(Node(PixelCoords(i,j)));
					counter++;
					isNode[i][j]=true;
					continue;	
				}
				if(	!voronoi[i-1][j-1] 		&& !voronoi[i][j-1] 	&& voronoi[i+1][j-1] && 
					!voronoi[i-1][j] 		&& voronoi[i][j] 		&& !voronoi[i+1][j] 	&&
					!voronoi[i-1][j+1] 		&& !voronoi[i][j+1] 	&& !voronoi[i+1][j+1]  && coverage[i][j]<127)
				{
					nodes.push_back(Node(PixelCoords(i,j)));
					counter++;
					isNode[i][j]=true;
					continue;	
				}
				if(	!voronoi[i-1][j-1] 		&& !voronoi[i][j-1] 	&& !voronoi[i+1][j-1] && 
					!voronoi[i-1][j] 		&& voronoi[i][j] 		&& voronoi[i+1][j] 	&&
					!voronoi[i-1][j+1] 		&& !voronoi[i][j+1] 	&& !voronoi[i+1][j+1]  && coverage[i][j]<127)
				{
					nodes.push_back(Node(PixelCoords(i,j)));
					counter++;
					isNode[i][j]=true;
					continue;	
				}
				if(	!voronoi[i-1][j-1] 		&& !voronoi[i][j-1] 	&& !voronoi[i+1][j-1] && 
					!voronoi[i-1][j] 		&& voronoi[i][j] 		&& !voronoi[i+1][j] 	&&
					!voronoi[i-1][j+1] 		&& !voronoi[i][j+1] 	&& voronoi[i+1][j+1]  && coverage[i][j]<127)
				{
					nodes.push_back(Node(PixelCoords(i,j)));
					counter++;
					isNode[i][j]=true;
					continue;	
				}
				if(	!voronoi[i-1][j-1] 		&& !voronoi[i][j-1] 	&& !voronoi[i+1][j-1] && 
					!voronoi[i-1][j] 		&& voronoi[i][j] 		&& !voronoi[i+1][j] 	&&
					!voronoi[i-1][j+1] 		&& voronoi[i][j+1] 	&& !voronoi[i+1][j+1]  && coverage[i][j]<127)
				{
					nodes.push_back(Node(PixelCoords(i,j)));
					counter++;
					isNode[i][j]=true;
					continue;	
				}
				if(	!voronoi[i-1][j-1] 		&& !voronoi[i][j-1] 	&& !voronoi[i+1][j-1] && 
					!voronoi[i-1][j] 		&& voronoi[i][j] 		&& !voronoi[i+1][j] 	&&
					voronoi[i-1][j+1] 		&& !voronoi[i][j+1] 		&& !voronoi[i+1][j+1]  && coverage[i][j]<127)
				{
					nodes.push_back(Node(PixelCoords(i,j)));
					counter++;
					isNode[i][j]=true;
					continue;	
				}
				if(	!voronoi[i-1][j-1] 		&& !voronoi[i][j-1] 	&& !voronoi[i+1][j-1] && 
					voronoi[i-1][j] 		&& voronoi[i][j] 		&& !voronoi[i+1][j] 	&&
					!voronoi[i-1][j+1] 		&& !voronoi[i][j+1] 		&& !voronoi[i+1][j+1]  && coverage[i][j]<127)
				{
					nodes.push_back(Node(PixelCoords(i,j)));
					counter++;
					isNode[i][j]=true;
					continue;
				}
				
				//	Check for voronoi cross paths
				
				unsigned char count=0;
				count=	voronoi[i-1][j-1]+voronoi[i][j-1]+voronoi[i+1][j-1]+ 
						voronoi[i-1][j]+voronoi[i][j]+voronoi[i+1][j]+
						voronoi[i-1][j+1]+voronoi[i][j+1]+voronoi[i+1][j+1];
				if(count>=4){
					//~ if(checkCrosspathNodes(i,j,voronoi)){
					if(checkCrosspathNodes(i,j)){
						nodes.push_back(Node(PixelCoords(i,j)));
						counter++;
						isNode[i][j]=true;
					}
					continue;
				}
			}
		}
	}
	
	ROS_INFO_NAMED("voronoi_nodes","[detect_nodes %d] Getting connected voronoi",__LINE__);
	getConnectedVoronoi(field,voronoi,map,mapHeight,mapWidth,robotPose,isNode,xMin,xMax,yMin,yMax);
	
	ROS_INFO_NAMED("voronoi_nodes","[detect_nodes %d] Fixing voronoi_nodes' neighbors",__LINE__);
	for(unsigned int i=0;i<nodes.size();i++){
		//~ insertNodeNeighbors(&nodes[i],nodes,isNode,voronoi,info,xMin,xMax,yMin,yMax,field,coverage);		
		insertNodeNeighbors(&nodes[i],isNode,voronoi,mapHeight, mapWidth,xMin,xMax,yMin,yMax, field, coverage);
	}
	
	//VoronoiNodes tempnodes = new VoronoiNodes(mapAttributes, this->coverage);
	NodesVector tempnodes;
	for(unsigned int i=0;i<nodes.size();i++){
		if(nodes[i].neigh.size()==1 && nodes[i].dist[0]==1) continue;
		tempnodes.nodes.push_back(nodes[i]);
	}
	tempnodes.nodes.swap(nodes);
	tempnodes.nodes.clear();
	for(unsigned int i=0;i<nodes.size();i++){
		Node temp=nodes[i];
		temp.neigh.clear();
		temp.dist.clear();
		temp.neighID.clear();
		for(unsigned int j=0;j<nodes[i].dist.size();j++){
			Node temp2(nodes[i].neigh[j]);
			//~ if(checkIfNodeInVoronodes(&temp2,nodes)){
			if(checkIfNodeInVoronodes(&temp2)){
				temp.neigh.push_back(nodes[i].neigh[j]);
				temp.dist.push_back(nodes[i].dist[j]);
			}
		}
		
		tempnodes.nodes.push_back(temp);
	}
	
	nodes.swap(tempnodes.nodes);
	int emptyNodeIndex;
	bool flag=false;
	for(unsigned int i=0;i<nodes.size();i++)
		if(nodes[i].neigh.size()==0){
			emptyNodeIndex=i;
			flag=true;
			break;
		}
	if(flag)
		nodes.erase(nodes.begin()+emptyNodeIndex);
	//cout<<"Found nodes neigbors \n\n";
}


//~ void VoronoiNodes::eliminateNodes(Voronodes *nodes,unsigned char **coverage){
void VoronoiNodes::eliminateNodes(void){
	
	bool changed;
	unsigned int dist;

	changed=true;
	while (changed){
		changed=false;
		for(unsigned int i=0;i<nodes.size();i++){
			if(coverage->coverage[nodes[i].p.getXCoord()][nodes[i].p.getYCoord()] > 127  && nodes[i].neigh.size()==2){
				if((nodes[i].neigh[0].getXCoord()!=nodes[i].neigh[1].getXCoord()) && (nodes[i].neigh[0].getYCoord()!=nodes[i].neigh[1].getYCoord())){
					//cout<<"\n o kombos "<<nodes[i].p.getXCoord()<<" , "<<nodes[i].p.getYCoord()<<" tha paraluftei \n";
					dist=nodes[i].dist[0]+nodes[i].dist[1];
					//cout<<"....."<<transformPixelCoordsToNode(nodes[i].neigh[0])->neigh.size()<<"....\n";
					for(unsigned int l=0;l<transformPixelCoordsToNode(nodes[i].neigh[0])->neigh.size();l++){
						if((transformPixelCoordsToNode(nodes[i].neigh[0])->neigh[l].getXCoord()==nodes[i].p.getXCoord()) && (transformPixelCoordsToNode(nodes[i].neigh[0])->neigh[l].getYCoord()==nodes[i].p.getYCoord())){
							transformPixelCoordsToNode(nodes[i].neigh[0])->neigh[l]=nodes[i].neigh[1];
							transformPixelCoordsToNode(nodes[i].neigh[0])->dist[l]=dist;
						}
					}
					
					for(unsigned int l=0;l<transformPixelCoordsToNode(nodes[i].neigh[1])->neigh.size();l++){
						if((transformPixelCoordsToNode(nodes[i].neigh[1])->neigh[l].getXCoord()==nodes[i].p.getXCoord()) && (transformPixelCoordsToNode(nodes[i].neigh[1])->neigh[l].getYCoord()==nodes[i].p.getYCoord())){
							transformPixelCoordsToNode(nodes[i].neigh[1])->neigh[l]=nodes[i].neigh[0];
							transformPixelCoordsToNode(nodes[i].neigh[1])->dist[l]=dist;
						}
					}
					
					nodes[i].neigh.clear();
					nodes.erase(nodes.begin()+i);
					//cout<<"eliminate done...\n";
					changed=true;
					break;
				}
			}
		}
	}
	
}


//~ void Voronodes::eliminateUnconnectedNodes(Voronodes &nodes){
void VoronoiNodes::eliminateUnconnectedNodes(void){
	
	std::vector< std::map<int,int> > node_groups;
	node_groups.clear();
	int correct_map=0;
	int idd;
	NodesVector totalNodes;
	totalNodes.nodes.clear();
	
	for(unsigned int i=0;i<nodes.size();i++){
		correct_map=node_groups.size();
		for(unsigned int j=0;j<node_groups.size();j++){
			if(node_groups[j].find(nodes[i].ID)!=node_groups[j].end()){
				correct_map=j;
				break;
			}
		}
		if((unsigned int)correct_map==node_groups.size()){
			std::map<int,int> temp;
			temp.insert( std::pair<int,int>(nodes[i].ID,0) );
			node_groups.push_back(temp);
			
		}
		for(unsigned int j=0;j<nodes[i].neighID.size();j++){
			if(node_groups[correct_map].find(nodes[i].neighID[j])==node_groups[correct_map].end()){
				idd=nodes[i].neighID[j];
				node_groups[correct_map].insert( std::pair<int,int>(idd,0) );
			}
		}
	}
	
	std::map<int,int>::iterator it,it2;
	bool done=false,innerdone=false;
	
	while(!done){
		innerdone=false;
		int source=0;
		int doublen=0;
		for(unsigned int i=0;i<node_groups.size();i++){
			source=i;
			for(it=node_groups[i].begin();it!=node_groups[i].end();it++){
				for(unsigned int j=i+1;j<node_groups.size();j++){
					doublen=j;
					if(node_groups[j].find(it->first)!=node_groups[j].end()){
						innerdone=true;
						break;	
					}
				}
				if(innerdone) break;
			}
			if(innerdone){
				for(it2=node_groups[doublen].begin();it2!=node_groups[doublen].end();it2++){
					if(node_groups[source].find(it2->first)==node_groups[source].end())
						node_groups[source].insert(std::pair<int,int>(it2->first,0));
				}
				node_groups.erase(node_groups.begin()+doublen);
				break;
			}
		}
		if(innerdone) continue;
		else done=true;
	}
	for(it=node_groups[0].begin();it!=node_groups[0].end();it++){
		totalNodes.nodes.push_back(nodes[it->first]);
	}
	
	nodes.swap(totalNodes.nodes);
	
}//end of eliminateUnconnectedNodes function



//~ void Voronodes::eliminateUncoveredNodesPaths(Voronodes *nodes,unsigned char **coverage){
void VoronoiNodes::eliminateUncoveredNodesPaths(void){

	std::vector<Identity> nodesIdentity;
	std::vector<std::vector<Connection> > a;
	std::vector<int> neihbour;
	std::vector<unsigned int> uncoveredNodes;
	std::vector<unsigned int> coveredNodes;
	NodesVector totalNodes;
	std::vector<PossiblePath> total_path;
	unsigned int source=0;
	unsigned int target=0;
	int infinity=9999;
	char current='c';
	char unvisited='u';
	char visited='v';
	int counter=0;
	std::vector<PossiblePath>::iterator i;
	std::vector<PossiblePath>::iterator j;
	std::vector<unsigned int>::iterator it;
	bool found=false;
	bool nonSame=false;
	coveredNodes.clear();
	uncoveredNodes.clear();
	
	for(unsigned int i=0;i<nodes.size();i++){
		if(coverage->coverage[nodes[i].p.getXCoord()][nodes[i].p.getYCoord()] < 127)
			uncoveredNodes.push_back(nodes[i].ID);
	}	
					
	for(unsigned int i=0;i<nodes.size();i++){
		a.push_back(std::vector<Connection>());
		for(unsigned int j=0;j<nodes[i].neigh.size();j++){
			a[i].push_back(Connection(nodes[i].neighID[j],nodes[i].dist[j]));
		}
	}
		
	for (unsigned int s=0;s<uncoveredNodes.size();s++){
		for (unsigned int t=0;t<uncoveredNodes.size();t++){
			if(t>s){
				int iden=0;
				neihbour.clear();
				nodesIdentity.clear();
				PossiblePath paths;
			
				for (unsigned int i=0;i<a.size();i++)
					neihbour.push_back(0);
		
				for (unsigned int i=0;i<a.size();i++)
					nodesIdentity.push_back(Identity(unvisited,infinity));
				
				source=uncoveredNodes[s];
				target=uncoveredNodes[t];

				paths.start_node=source;
				paths.end_node=target;
				nodesIdentity[source].state=current;
				nodesIdentity[source].dist=0;
				while(nodesIdentity[target].state!=visited){
					for (unsigned int i=0;i<a.size();i++){
						for (unsigned int j=0;j<a[i].size();j++){
							if(nodesIdentity[i].state==current && nodesIdentity[a[i].at(j).id].state!=visited){
								neihbour[a[i].at(j).id]=nodesIdentity[i].dist+a[i].at(j).weight;
									if(neihbour[a[i].at(j).id]<nodesIdentity[a[i].at(j).id].dist){
										nodesIdentity[a[i].at(j).id].dist=neihbour[a[i].at(j).id];
									}
							}
						}
						bool all_visited=true;
						for (unsigned int j=0;j<a[i].size();j++){
							if(nodesIdentity[i].state==current && nodesIdentity[a[i].at(j).id].state!=visited)
								all_visited=false;
						}
						if(all_visited==true && nodesIdentity[i].state==current){
							int min=100000;
							nodesIdentity[i].state=visited;
								for (unsigned int k=0;k<a.size();k++){
									if(nodesIdentity[k].state!=visited && nodesIdentity[k].dist<min){
										min=nodesIdentity[k].dist;
										iden=k;
									}
								}
						}
						int min=100000;
						if(nodesIdentity[i].state==current){
							nodesIdentity[i].state=visited;
								for (unsigned int k=0;k<a[i].size();k++){
									if(nodesIdentity[a[i].at(k).id].state!=visited && nodesIdentity[a[i].at(k).id].dist<min){
										min=nodesIdentity[a[i].at(k).id].dist;	
										iden=a[i].at(k).id;
									}
								}
						}
					}
					if(nodesIdentity[target].state!=visited)
						nodesIdentity[iden].state=current;
				}
				if(nodesIdentity[target].state==visited){
					paths.total_path.push_back(target);
					for (unsigned int i=0;i<a.size();i++){
						if(i!=target)
							nodesIdentity[i].state=unvisited;
					}
					int min=100000;
					for (unsigned int j=0;j<a[target].size();j++){
						if(nodesIdentity[a[target].at(j).id].dist + a[target].at(j).weight<(unsigned int)min){
							min=nodesIdentity[a[target].at(j).id].dist + a[target].at(j).weight;
							iden=a[target].at(j).id;		
						}
					}
					nodesIdentity[iden].state=current;
					paths.total_path.push_back(iden);
					while(nodesIdentity[source].state!=current){
						for (unsigned int i=0;i<a.size();i++){
							for (unsigned int j=0;j<a[i].size();j++){
								if(nodesIdentity[i].state==current && nodesIdentity[a[i].at(j).id].dist<min && nodesIdentity[a[i].at(j).id].state!=visited){
									min=nodesIdentity[a[i].at(j).id].dist;
									iden=a[i].at(j).id;
								}
							}
						}
						nodesIdentity[iden].state=current;
						paths.total_path.push_back(iden);
					}
				}
				paths.cost=nodesIdentity[target].dist;
				total_path.push_back(paths);
				counter++;
			}
		}
	}

	for(i = total_path.begin(); i != total_path.end();++i){
		for(int j=(*i).total_path.size()-1;j>-1;j--){
			if((*i).total_path.at(j)!=(*i).start_node && (*i).total_path.at(j)!=(*i).end_node){
				found=false;
				for(unsigned int k=0;k<uncoveredNodes.size();k++){
					if((*i).total_path.at(j)==(int)uncoveredNodes[k]){
						found=true;
						break;
					}
				}
				if(!found){
					if(coveredNodes.size()==0)
						coveredNodes.push_back((*i).total_path.at(j));
					nonSame=false;
					for(unsigned int k=0;k<coveredNodes.size();k++){
						if((*i).total_path.at(j)==(int)coveredNodes[k]){
							nonSame=true;
							break;
						}
					}
					if(!nonSame)
						coveredNodes.push_back((*i).total_path.at(j));
				}
			}
		}
	}
    
    for(unsigned int i=0;i<nodes.size();i++){
		 for(unsigned int j=0;j<uncoveredNodes.size();j++){
			if(nodes[i].ID==uncoveredNodes[j])
				totalNodes.nodes.push_back(Node(PixelCoords(nodes[i].p.getXCoord(),nodes[i].p.getYCoord())));
		}
		for(unsigned int j=0;j<coveredNodes.size();j++){
			if(nodes[i].ID==coveredNodes[j])
				totalNodes.nodes.push_back(Node(PixelCoords(nodes[i].p.getXCoord(),nodes[i].p.getYCoord())));
		}
	}
	
	for(unsigned int j=0;j<totalNodes.nodes.size();j++){
		for(unsigned int i=0;i<nodes.size();i++){
			if(totalNodes.nodes[j].p.getXCoord()==nodes[i].p.getXCoord() && totalNodes.nodes[j].p.getYCoord()==nodes[i].p.getYCoord()){		
				for(unsigned int k=0;k<nodes[i].neigh.size();k++){
					for(unsigned int l=0;l<uncoveredNodes.size();l++){
						if(nodes[i].neighID[k]==uncoveredNodes[l]){
							totalNodes.nodes[j].neigh.push_back(nodes[i].neigh[k]);
							totalNodes.nodes[j].dist.push_back(nodes[i].dist[k]);
						}
					}
				}
				for(unsigned int k=0;k<nodes[i].neigh.size();k++){
					for(unsigned int l=0;l<coveredNodes.size();l++){
						if(nodes[i].neighID[k]==coveredNodes[l]){
							totalNodes.nodes[j].neigh.push_back(nodes[i].neigh[k]);
							totalNodes.nodes[j].dist.push_back(nodes[i].dist[k]);
						}
					}
				}
			}
		}
	}

	nodes.swap(totalNodes.nodes);

}//end of eliminateUncoveredNodesPaths function


PixelCoords VoronoiNodes::findCoordsFromId(unsigned int id){
	for(unsigned int i=0;i<nodes.size();i++){
		if(nodes[i].ID==id)
			return nodes[i].p;
	}
	return PixelCoords(-1,-1);
}


//~ void VoronoiNodes::getConnectedVoronoi(int **field,bool **voronoi,unsigned char **map,MapInfo info,Transformation robotPose,Voronodes &nodes,bool **isNode,int xMin,int xMax,int yMin,int yMax){
void VoronoiNodes::getConnectedVoronoi(int **field,bool **voronoi,unsigned char **map, int mapHeight, int mapWidth, Transformation robotPose,bool **isNode,int xMin,int xMax,int yMin,int yMax){
//void VoronoiNodes::getConnectedVoronoi(unsigned char **map, Transformation robotPose){
	
	PixelCoords rpose(robotPose.dx+MAP_SIZE/2,robotPose.dy+MAP_SIZE/2);
	
	PixelCoords closestOnVoronoi=getOptClosest(field,voronoi, map, mapHeight, mapWidth, rpose);
	//PixelCoords closestOnVoronoi=getOptClosest(map, rpose);
	float currDist;
	std::vector<PixelCoords> currCheck, nextCheck;

	for(int i=xMin;i<xMax;i++)
	{   for(int j=yMin;j<yMax;j++)
		{   if(!voronoi[i][j])
			{   field[i][j] = 0;
			}
			else
			{   field[i][j] = -1;
			}
		}
	}
	field[closestOnVoronoi.getXCoord()][closestOnVoronoi.getYCoord()] = 1;
	currCheck.push_back(closestOnVoronoi);
	
	std::vector<Node> toBeKept;
	toBeKept.clear();
	
	unsigned int counter=0;
	do
	{   
		for(unsigned int k = 0;k<currCheck.size();k++)
		{   int i = currCheck[k].getXCoord();
			int j = currCheck[k].getYCoord();
			currDist = (float)(field[i][j]);
			if(isNode[i][j]){
				toBeKept.push_back(Node(PixelCoords(i,j)));
			}
			if(i > 0)
			{   
				if(voronoi[i-1][j]){
					if((field[i-1][j] < 0) || (field[i-1][j] > (currDist + 1)))
					{   
						field[i-1][j] = (currDist + 1);
						nextCheck.push_back(PixelCoords(i-1, j));
					}
				}
			}
			if(j > 0)
			{  
				if(voronoi[i][j-1]){
					if((field[i][j-1] < 0) || (field[i][j-1] > (currDist + 1)))
					{   
						field[i][j-1] = (currDist + 1);
						nextCheck.push_back(PixelCoords(i, j-1));
					}
				}
			}
			if(i < (mapHeight - 1))
			{   
				if(voronoi[i+1][j]){
					if((field[i+1][j] < 0) || (field[i+1][j] > (currDist + 1)))
					{   
						field[i+1][j] = (currDist + 1);
						nextCheck.push_back(PixelCoords(i+1, j));
					}
				}
			}
			if(j < (mapWidth - 1))
			{   
				if(voronoi[i][j+1]){
					if((field[i][j+1] < 0) || (field[i][j+1] > (currDist + 1)))
					{   
						field[i][j+1] = (currDist + 1);
						nextCheck.push_back(PixelCoords(i, j+1));
					}
				}
			}
			if(i > 0 && j > 0)
			{   
				if(voronoi[i-1][j-1]){
					if((field[i-1][j-1] < 0) || (field[i-1][j-1] > (currDist + 1)))
					{   
						field[i-1][j-1] = (currDist + 1);
						nextCheck.push_back(PixelCoords(i-1, j-1));
					}
				}
			}
			if(i > 0 && j < mapWidth)
			{   
				if(voronoi[i-1][j+1]){
					if((field[i-1][j+1] < 0) || (field[i-1][j+1] > (currDist + 1)))
					{   
						field[i-1][j+1] = (currDist + 1);
						nextCheck.push_back(PixelCoords(i-1, j+1));
					}
				}
			}
			if(i < mapHeight && j > 0)
			{   
				if(voronoi[i+1][j-1]){
					if((field[i+1][j-1] < 0) || (field[i+1][j-1] > (currDist + 1)))
					{   
						field[i+1][j-1] = (currDist + 1);
						nextCheck.push_back(PixelCoords(i+1, j-1));
					}
				}
			}
			if(i < mapHeight && j < mapWidth)
				{   
					if(voronoi[i+1][j+1]){
					if((field[i+1][j+1] < 0) || (field[i+1][j+1] > (currDist + 1)))
					{   
						field[i+1][j+1] = (currDist + 1);
						nextCheck.push_back(PixelCoords(i+1, j+1));
					}
				}
			}
		}
		currCheck = nextCheck;
		nextCheck.clear();
		counter++;
	}while(currCheck.size() > 0);	
	nodes.swap(toBeKept);
}


PixelCoords VoronoiNodes::getOptClosest(int **field,bool **voronoi, unsigned char **map, int mapHeight, int mapWidth, PixelCoords position){
//PixelCoords VoronoiNodes::getOptClosest(unsigned char** map, PixelCoords position){
	bool stop = false;
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
				field[i][j] = 0;
			else
				field[i][j] = -1;
		}
	}
	field[position.getXCoord()][position.getYCoord()] = 1;
	currCheck.push_back(position);

	while(!stop)
	{   for(unsigned int k=0;k<currCheck.size();k++)
		{   int i = (currCheck[k]).getXCoord();
			int j = (currCheck[k]).getYCoord();
			currDist = field[i][j];

			if(i > 0)
			{   if(!(voronoi[i-1][j]))
				{   if((field[i-1][j] < 0) ||
					   (field[i-1][j] > (currDist + 1)))
					{   field[i-1][j] = (currDist + 1);
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
				{   if((field[i][j-1] < 0) ||
					   (field[i][j-1] > (currDist + 1)))
					{   field[i][j-1] = (currDist + 1);
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
				{   if((field[i+1][j] < 0) ||
					   (field[i+1][j] > (currDist + 1)))
					{   field[i+1][j] = (currDist + 1);
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
				{   if((field[i][j+1] < 0) ||
					   (field[i][j+1] > (currDist + 1)))
					{   field[i][j+1] = (currDist + 1);
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


//~ void Voronodes::insertNodeNeighbors(Node *n,Voronodes &nodes,bool **isNode,bool **voronoi,MapInfo info,int xMin,int xMax,int yMin,int yMax,int **field,unsigned char **coverage){
//void VoronoiNodes::insertNodeNeighbors(Node *n){
void VoronoiNodes::insertNodeNeighbors(Node *n,bool **isNode,bool **voronoi, int mapHeight, int mapWidth,int xMin,int xMax,int yMin,int yMax,int **field,unsigned char **coverage){
	float currDist;
	std::vector<PixelCoords> currCheck, nextCheck;
	
	for(int i=xMin;i<xMax;i++)
	{   for(int j=yMin;j<yMax;j++)
		{   if(!voronoi[i][j])
			{   field[i][j] = 0;
			}
			else
			{   field[i][j] = -1;
			}
		}
	}
	field[n->p.getXCoord()][n->p.getYCoord()] = 1;
	n->neigh.clear();
	n->dist.clear();
	currCheck.push_back(n->p);
	unsigned int counter=0;
	do
	{   
		for(unsigned int k = 0;k<currCheck.size();k++)
		{   int i = currCheck[k].getXCoord();
			int j = currCheck[k].getYCoord();
			currDist = (float)(field[i][j]);
			
			if(isNode[i][j] && !(i==n->p.getXCoord() && j==n->p.getYCoord())){
				if(checkForNeigbor(currCheck[k],n->neigh)) continue;
				n->neigh.push_back(currCheck[k]);
				n->dist.push_back(counter);
				continue;
			}
			else if(!isNode[i][j] && !(i==n->p.getXCoord() && j==n->p.getYCoord())){
				if(isNode[i-1][j-1] && !(i-1==n->p.getXCoord() && j-1==n->p.getYCoord())) {
					if(checkForNeigbor(PixelCoords(i-1,j-1),n->neigh)) continue;
					n->neigh.push_back(PixelCoords(i-1,j-1));
					n->dist.push_back(counter);
					continue;
				}
				if(isNode[i-1][j] && !(i-1==n->p.getXCoord() && j==n->p.getYCoord())){
					if(checkForNeigbor(PixelCoords(i-1,j),n->neigh)) continue;
					n->neigh.push_back(PixelCoords(i-1,j));
					n->dist.push_back(counter);
					continue;
				}
				if(isNode[i-1][j+1] && !(i-1==n->p.getXCoord() && j+1==n->p.getYCoord())){
					if(checkForNeigbor(PixelCoords(i-1,j+1),n->neigh)) continue;
					n->neigh.push_back(PixelCoords(i-1,j+1));
					n->dist.push_back(counter);
					continue;
				}
				if(isNode[i][j-1] && !(i==n->p.getXCoord() && j-1==n->p.getYCoord())){
					if(checkForNeigbor(PixelCoords(i,j-1),n->neigh)) continue;
					n->neigh.push_back(PixelCoords(i,j-1));
					n->dist.push_back(counter);
					continue;
				}
				if(isNode[i][j+1] && !(i==n->p.getXCoord() && j+1==n->p.getYCoord())){
					if(checkForNeigbor(PixelCoords(i,j+1),n->neigh)) continue;
					n->neigh.push_back(PixelCoords(i,j+1));
					n->dist.push_back(counter);
					continue;
				}
				if(isNode[i+1][j-1] && !(i+1==n->p.getXCoord() && j-1==n->p.getYCoord())){
					if(checkForNeigbor(PixelCoords(i+1,j-1),n->neigh)) continue;
					n->neigh.push_back(PixelCoords(i+1,j-1));
					n->dist.push_back(counter);
					continue;
				}
				if(isNode[i+1][j] && !(i+1==n->p.getXCoord() && j==n->p.getYCoord())){
					if(checkForNeigbor(PixelCoords(i+1,j),n->neigh)) continue;
					n->neigh.push_back(PixelCoords(i+1,j));
					n->dist.push_back(counter);
					continue;
				}
				if(isNode[i+1][j+1] && !(i+1==n->p.getXCoord() && j+1==n->p.getYCoord())){
					if(checkForNeigbor(PixelCoords(i+1,j+1),n->neigh)) continue;
					n->neigh.push_back(PixelCoords(i+1,j+1));
					n->dist.push_back(counter);
					continue;
				}
			}

			if(i > 0)
			{   
				if(voronoi[i-1][j]){
					if((field[i-1][j] < 0) || (field[i-1][j] > (currDist + 1)))
					{   
						field[i-1][j] = (currDist + 1);
						nextCheck.push_back(PixelCoords(i-1, j));
					}
				}
			}
			if(j > 0)
			{  
				if(voronoi[i][j-1]){
					if((field[i][j-1] < 0) || (field[i][j-1] > (currDist + 1)))
					{   
						field[i][j-1] = (currDist + 1);
						nextCheck.push_back(PixelCoords(i, j-1));
					}
				}
			}
			if(i < (mapHeight - 1))
			{   
				if(voronoi[i+1][j]){
					if((field[i+1][j] < 0) || (field[i+1][j] > (currDist + 1)))
					{   
						field[i+1][j] = (currDist + 1);
						nextCheck.push_back(PixelCoords(i+1, j));
					}
				}
			}
			if(j < (mapWidth - 1))
			{   
				if(voronoi[i][j+1]){
					if((field[i][j+1] < 0) || (field[i][j+1] > (currDist + 1)))
					{   
						field[i][j+1] = (currDist + 1);
						nextCheck.push_back(PixelCoords(i, j+1));
					}
				}
			}
			if(i > 0 && j > 0)
			{   
				if(voronoi[i-1][j-1]){
					if((field[i-1][j-1] < 0) || (field[i-1][j-1] > (currDist + 1)))
					{   
						field[i-1][j-1] = (currDist + 1);
						nextCheck.push_back(PixelCoords(i-1, j-1));
					}
				}
			}
			if(i > 0 && j < mapWidth)
			{   
				if(voronoi[i-1][j+1]){
					if((field[i-1][j+1] < 0) || (field[i-1][j+1] > (currDist + 1)))
					{   
						field[i-1][j+1] = (currDist + 1);
						nextCheck.push_back(PixelCoords(i-1, j+1));
					}
				}
			}
			if(i < mapHeight && j > 0)
			{   
				if(voronoi[i+1][j-1]){
					if((field[i+1][j-1] < 0) || (field[i+1][j-1] > (currDist + 1)))
					{   
						field[i+1][j-1] = (currDist + 1);
						nextCheck.push_back(PixelCoords(i+1, j-1));
					}
				}
			}
			if(i < mapHeight && j < mapWidth)
				{   
					if(voronoi[i+1][j+1]){
					if((field[i+1][j+1] < 0) || (field[i+1][j+1] > (currDist + 1)))
					{   
						field[i+1][j+1] = (currDist + 1);
						nextCheck.push_back(PixelCoords(i+1, j+1));
					}
				}
			}
		}
		currCheck = nextCheck;
		nextCheck.clear();
		counter++;
	}while(currCheck.size() > 0);
	//cout<<"Found "<<n->neigh.size()<<" neighbors\n";
}



void VoronoiNodes::manipulateNodes(){
	
	if(nodes.size()<2) return;
	
	assignIDs();

	ROS_INFO_NAMED("voronoi_nodes","[manipulate_nodes %d] Eliminating unconnected nodes",__LINE__);
	eliminateUnconnectedNodes();
	assignIDs();
	
	ROS_INFO_NAMED("voronoi_nodes","[manipulate_nodes %d] Eliminating nodes based on coverage",__LINE__);
	eliminateNodes();
	assignIDs();
	
	ROS_INFO_NAMED("voronoi_nodes","[manipulate_nodes %d] Eliminating nodes that do not participate in shortest paths connecting unconnected nodes",__LINE__);
	eliminateUncoveredNodesPaths();
	assignIDs();

}


//~ Voronode* transformcPixelCoordsToVoronoid(Voronodes *nodes,PixelCoords c){
Node* VoronoiNodes::transformPixelCoordsToNode(PixelCoords c){
	for(unsigned int i=0;i<nodes.size();i++){
		if((nodes[i].p.getXCoord()==c.getXCoord()) && (nodes[i].p.getYCoord()==c.getYCoord()))
			return &(nodes[i]);
	}
	return NULL;
}


