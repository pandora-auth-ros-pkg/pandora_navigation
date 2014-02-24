#include "navigation/path_planner/path_generator/tree_path_generator/tree_node.h"

TreeNode::TreeNode(PixelCoords a,int id_a){
	p=a;
	ID=id_a;
}


bool TreeNode::eraseChild(TreeNode * child){		
	for(unsigned int i=0;i<children.size();i++){
		if(children[i]->p==child->p){
			children.erase(children.begin()+i);
			return true;
		}
	}
	return false;
}


void TreeNode::print(void){
	std::cout<<"[";
	if(ID==0) std::cout<<"Root]\t";
	else std::cout<<parent->ID<<"]\t";
	std::cout<<"ID="<<ID<<"\t ";
	for(unsigned int i=0;i<children.size();i++)
		std::cout<<children[i]->ID<<" ";
	std::cout<<"\n";
	for(unsigned int i=0;i<children.size();i++)
		children[i]->print();
}
