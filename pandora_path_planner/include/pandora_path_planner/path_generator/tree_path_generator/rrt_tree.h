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

#ifndef RRT_TREE_H
#define RRT_TREE_H

#include "pandora_navigation_common/misc/rrt_tree_defines.h"
#include "pandora_navigation_common/voronoi/voronoi.h"
#include "tree_node.h"
#include "vector"
#include "iostream"
#include "stdlib.h"

class RRTTree{

public:
	
	MapAttributes* mapAttributes;
	
	Voronoi* voronoi;
	
	int ID;									//!< The id of the Tree
	
	TreeNode *root;							//!< The starting node of the Tree
	
	std::map <int,TreeNode *> checkNodes; 	//!< All the nodes of the tree
	
	std::vector <float> cost;				//!< The cost of a tree node according to it's distance


	//~ RRTTree(MapAttributes* mapAttr): voronoi(mapAttr,true) {}
	
	 /**
	@brief  constructor
	@param root : the root of the tree
	**/
	//RRTTree(PixelCoords root);
	RRTTree(PixelCoords root, MapAttributes* mapAttr, Voronoi* vor);
	
	/**
	@brief Finds the path in a tree
	@param end [int] : the last node inserted in tree
	@param path [std::vector<PixelCoords> &] : the path that is constructed
	@return void
	**/
	void choosePath(int end,std::vector<PixelCoords> &path);
	
	/**
	@brief Deletes the tree
	@param void
	@return void
	**/
	void deleteNodes(void);
	
	/**
	@brief prints a Tree
	@param void
	@return void
	**/
	void print(void);

	/**
	@brief expands a tree
	@param prevxMax	: The previous limits of the map - Maximum X
	@param prevxMin : The previous limits of the map - Minimum X
	@param prevyMin : The previous limits of the map - Maximum Y
	@param prevyMin : The previous limits of the map - Minimum Y
	@param brushCell : Helping matrix - Holds the brushfire values at the voronoi construction

	@return void
	**/
	//~ TODO: PROBABLY USE map attribute HERE. READ THEM FROM treePLANER
	//~ bool expand(int prevxMax,int prevxMin,int prevyMax,int prevyMin,float **brushCell,PixelCoords goal);
	bool expand(PixelCoords goal);
	

};		


#endif



