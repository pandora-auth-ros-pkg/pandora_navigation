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



