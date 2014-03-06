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

#ifndef PARTITION_GRAPG_NODES_H
#define PARTITION_GRAPG_NODES_H

#include "nodes_vector.h"
#include "pandora_navigation_common/voronoi/voronoi.h"
#include "vector"
#include "ros/ros.h"
#include "pandora_navigation_common/misc/partition_graph_nodes_defines.h"

class PartitionGraphNodes: public NodesVector{
	
private:
	
	//~ float** brushCell;
	Voronoi* voronoi;
	
	int** decomp;		//!< The nodes matrix
	
	std::vector<int> targetNeighbors;			//!<Vector holding target neighbors
	
	std::vector<int> robotNeighbors;			//!<Vector holding robot neighbors
	
	std::vector<PixelCoords> uniformPartition;
	
	//~ void performIncrementalPartition(vector <PixelCoords> &uniformPartition, int xmax, int xmin,int ymax, int ymin,unsigned char **map,float **brushCell,int **decomp);
	//~ void performIncrementalPartition(unsigned char **map);
	void performIncrementalPartition( int xmax, int xmin,int ymax, int ymin,unsigned char **map,float **brushCell );
	
	void reinitialiseIncrementalPartition(void);
	
public:
	
	//~ partitionGraphNodes(void);
	PartitionGraphNodes(MapAttributes* mapAttr, Voronoi* vor);
	
	
	void createIncrementalPartition(PixelCoords startPos, PixelCoords target);
	
	//~ TODO:possibly edit to work with Node as argument
	void eliminateNode(int ID,bool keepPosition);
	
	int insertNodeInPartition(Node currNode);

	
	// Η computeDistanceFrom που θα μπει πλέον;
};


#endif
