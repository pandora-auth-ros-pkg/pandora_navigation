#ifndef TREE_GENERATOR_TREE_NODE
#define TREE_GENERATOR_TREE_NODE


#include "vector"
#include "pandora_navigation_common/misc/pixelcoords.h"
#include "iostream"
#include "map"

class TreeNode{
		
public:


	PixelCoords p;							//!< The coordinates of the treeNode
		
	int ID;									//!< The ID of the treeNode

	TreeNode *parent;						//!< The pointer of the parent of the treeNode
	
	std::vector<TreeNode *> children;		//!< Vector of the children of the treeNode
	
	std::vector<float> dist;				//!< The distance of one node to another
	
	TreeNode(PixelCoords a,int id_a);
	 
	TreeNode();	

	bool eraseChild(TreeNode * child);

	void print(void);

	//not sore if these are needed for now
	/*
	 *void setP(PixelCoords pp) {p=pp;};
	 *PixelCoords getP() {return p;};
	 *
	 *void setID(int id) {ID=id;};
	 *int getID() {return ID;};
	 */
};

#endif
