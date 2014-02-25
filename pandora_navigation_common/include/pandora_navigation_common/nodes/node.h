#ifndef NAVIGATION_NODE
#define NAVIGATION_NODE


#include "vector"
#include "weight.h"
#include "pandora_navigation_common/misc/pixelcoords.h"
#include <boost/serialization/vector.hpp>

class Node{

public:
	
	PixelCoords p;						//!< The coordinates of the node
	bool visited;						//!< True if the node has been visited
	unsigned int ID;					//!< The node's ID
	int parent;
	
	std::vector<PixelCoords> neigh;		//!< Vector of the neighbors of the node (coords)
	std::vector<unsigned int> neighID;	//!< Vector of the neighbors' IDs
	std::vector<float> dist;			//!< Vector of the distances of the neighbors
	std::vector<unsigned int> colour;	//!< Not used
	
	Weight w;							//!< The node's weight
	
	std::vector<PixelCoords> path;
	


	// Συναρτήσεις του struct Node 
	Node(PixelCoords a) {p=a;}   
	Node(PixelCoords a,unsigned int IDt) {p=a; ID=IDt;}
	Node(void){}           
	
	//~ unsigned int findClosestNeighbor(void);  
	void makeNeighbor(Node &a);		
	//~ bool isNeighbor(Node &a);	
	
	
	template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
			ar & p;
			ar & visited;
			ar & ID;
			ar & parent;
			ar & neigh;
			//~ for(int ii=0;ii<neigh.size();ii++){
					//~ ar & neigh[ii];
				//~ }
			
			ar & neighID;
			ar & dist;
			ar & w;
	}
		
	
};

#endif
