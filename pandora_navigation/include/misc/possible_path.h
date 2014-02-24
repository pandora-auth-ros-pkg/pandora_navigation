#ifndef POSIIBLE_PATH_H
#define POSIIBLE_PATH_H

/*! \struct Possible_path
    \brief  Possible path between two nodes
*/
class PossiblePath{
public:
	int start_node;				//!< The start_node of the path
	int end_node;				//!< The end_node of the path
	std::vector<int> total_path;//!< The path
	int cost;					//!< The path's cost
};


#endif

