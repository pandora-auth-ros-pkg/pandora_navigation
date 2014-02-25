#ifndef NODES_VECTOR_H
#define NODES_VECTOR_H

#include "node.h"
#include "pandora_navigation_common/map/map_attributes.h"
#include <boost/serialization/vector.hpp>
//#include "vector"

class NodesVector{
	
	protected:
		
		//~ std::vector<Voronode> nodes;
		
	public:
		
		NodesVector(void){};
		std::vector<Node> nodes;
		MapAttributes* mapAttributes;
		
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version)
		{
				//~ for(int ii=0;ii<nodes.size();ii++){
					//~ ar & nodes[ii];
				//~ }
				ar & nodes;

		}
		
		
};


#endif


