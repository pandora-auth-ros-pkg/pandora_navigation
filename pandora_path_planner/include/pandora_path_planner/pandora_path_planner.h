#include "pandora_path_planner/global_path_planner.h"

#include "pandora_path_planner/path_generator/partition_graph_path_generator/partition_graph_path_generator.h"
#include "pandora_path_planner/path_generator/tree_path_generator/tree_path_generator.h"
#include "pandora_path_planner/path_generator/voronoi_path_generator/voronoi_path_generator.h"
#include "geometry_msgs/PoseStamped.h"
#include "pandora_navigation_common/map/map_attributes.h"

class PandoraPathPlanner: public GlobalPathPlanner{

public:
	
	Voronoi voronoi;
	
	MapAttributes* mapAttributes;
	VoronoiPathGenerator voronoiPathGenerator;
	TreePathGenerator treePathGenerator;
	PartitionGraphPathGenerator partitionGraphPathGenerator;
	//~ bool inTargetSelector;
	
public:
	
	//~ PandoraPathPlanner(void);
	PandoraPathPlanner(MapAttributes* mapAttr);
	
	
	bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
	
	//targetSelector path planner to return path for all possible targets
	void getTargetSelectorPaths(NodesVector* nodes);
	
};

