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
