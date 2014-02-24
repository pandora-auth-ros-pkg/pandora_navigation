#include "navigation/path_planner/path_generator/tree_path_generator/rrt_tree.h"

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


