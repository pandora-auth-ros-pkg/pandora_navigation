#ifndef PARTITION_GRAPH_PATH_GENERATOR_H
#define PARTITION_GRAPH_PATH_GENERATOR_H


#include "pandora_navigation_common/nodes/partition_graph_nodes.h"
#include "pandora_path_planner/path_generator/path_generator.h"


class PartitionGraphPathGenerator: public PathGenerator{
	
public:
	
	PartitionGraphNodes partitionNodes;
	
	
	
	//~ void PartitionGraphPathGenerator(void);
	PartitionGraphPathGenerator(MapAttributes* mapAttr,Voronoi *v);
	
	bool generatePath(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
	
	bool generatePath(NodesVector* nodes);
};


#endif


