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

#include "pandora_path_planner/path_generator/tree_path_generator/rrt_tree.h"

//~ RRTTree::RRTTree(PixelCoords node, MapAttributes* mapAttr): voronoi(mapAttr, true){
RRTTree::RRTTree(PixelCoords node, MapAttributes* mapAttr, Voronoi* vor){
	
	mapAttributes = mapAttr;
	voronoi = vor;
	
	root= new TreeNode(node,0);
	root->parent=NULL;
	root->p=node;
	checkNodes.insert(std::pair<int,TreeNode *>(0,root));
	cost.push_back(0);
}


void RRTTree::choosePath(int end,std::vector<PixelCoords> &path){
	path.clear();
	TreeNode *qNode=checkNodes[end];
	path.push_back(qNode->p);
	qNode=qNode->parent;
	while(1){
		if(qNode==NULL)
			break;
		path.push_back(qNode->p);
		qNode=qNode->parent;
		
	}
}


void RRTTree::deleteNodes(void){
	//~ for(unsigned int i=0;i<checkNodes.size();i++){
		//~ std::cout << checkNodes[i]->ID << "\n";
	//~ }
	for(unsigned int i=0;i<checkNodes.size();i++){
		checkNodes[i]->children.clear();
		delete checkNodes[i];
	}
}


//~ bool RRTTree::expand(int prevxMax,int prevxMin,int prevyMax,int prevyMin,float **brushCell,PixelCoords goal){
bool RRTTree::expand(PixelCoords goal){
	PixelCoords qRand,newNode;
	TreeNode *qNear=root,*qNew;
	float minDist=INFINITY_DISTANCE;
	float angle=0;
	float dist=rand()%EXPAND_DIST;
	
	int xRand=mapAttributes->prevxMin+rand()%(mapAttributes->prevxMax-mapAttributes->prevxMin);
	int yRand=mapAttributes->prevyMin+rand()%(mapAttributes->prevyMax-mapAttributes->prevyMin);
	qRand=PixelCoords(xRand,yRand);
	
	for(unsigned int i=0;i<checkNodes.size();i++){
		if(checkNodes[i]->p.computeSqrDistFrom(qRand)<minDist){
			minDist=checkNodes[i]->p.computeSqrDistFrom(qRand);
			qNear=checkNodes[i];
		}
	}
	angle=atan2((qRand.getYCoord()-qNear->p.getYCoord()),(qRand.getXCoord()-qNear->p.getXCoord()));
	int xNew=qNear->p.getXCoord()+dist*cos(angle);
	int yNew=qNear->p.getYCoord()+dist*sin(angle);

	if(voronoi->brushCell[xNew][yNew]>WALL_DISTANCE_SIMPLE_RRT){
		newNode=PixelCoords(xNew,yNew);
		qNew=new TreeNode(newNode,checkNodes.size());
		qNew->parent=qNear;
		qNear->children.push_back(qNew);
		checkNodes.insert(std::pair<int,TreeNode *>(qNew->ID,qNew));
		if(newNode.computeDistanceFrom(goal)<10)
			return true;
	}

	return false;
}


void RRTTree::print(void){
	std::cout<<"Tree :\n";
	for(unsigned int i=0;i<checkNodes.size();i++){
		if(checkNodes[i]->parent==NULL){
			std::cout<<"ID = "<<checkNodes[i]->ID<<"  "<< "Children = ";
		}
		else {
			std::cout<<"ID = "<<checkNodes[i]->ID<<"   Parent = "<<checkNodes[i]->parent->ID<<"  "<< "Children = ";	
		}
		for(unsigned int j=0;j<checkNodes[i]->children.size();j++){
			std::cout<<checkNodes[i]->children[j]->ID<<"  ";
		}
		
		std::cout<<" \n";
	}
}


