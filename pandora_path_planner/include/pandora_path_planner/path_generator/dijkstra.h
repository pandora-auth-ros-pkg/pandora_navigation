#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include "pandora_navigation_common/nodes/nodes_vector.h"
#include "pandora_navigation_common/misc/dijkstra_defines.h"
#include "set"
#include "vector"



class Dijkstra {
public:

	Dijkstra (void){}
	virtual ~Dijkstra (){}

	//TODO: possible error. check polymorphism errors (functions needed not in NodesVector)
	//~ TODO: change to work with node's internal path
	void findShortestPath(Node startPos,Node targetPos,NodesVector &graphNodes,std::vector<PixelCoords> &shortestPath);
	
	void findShortestPathAStar(Node startPos,Node targetPos,NodesVector &graphNodes,std::vector<PixelCoords> &shortestPath);
	
private:
	
	bool checkIfNodesAreConnected(NodesVector &nodes,int id_a,int id_b);
	
	
};


#endif


