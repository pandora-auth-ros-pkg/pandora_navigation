#include "nodes/node.h"

/**
@brief
Connects one Node with another

@param a one of the Nodes

@return void
**/

void Node::makeNeighbor(Node &a){
	if(a.ID==this->ID) return;
	for(unsigned int i=0;i<neigh.size();i++){
		if(neighID[i]==a.ID)
			return;
	}
	float dist=0;
	path.clear();
	dist=p.computeDistanceFrom(a.p);
	neigh.push_back(a.p);
	neighID.push_back(a.ID);
	this->dist.push_back(dist);
	
	a.neigh.push_back(p);
	a.neighID.push_back(ID);
	a.dist.push_back(dist);
	
	
}
