#include "closest_unexplored_target_selector.h"
#include "nodes/voronoi_nodes.h"
#include "navigation/path_planner/pandora_path_planner.h"
//~ #include <map>

//~ #include <vector>

//~ TODO: needs to update coverage

class ExplorationTargetSelector: public TargetSelector{
	
//~ private:
public:
	
	//~ Voronoi voronoiDiagram; is now in voronoiNodes
	VoronoiNodes voronoiNodes;
	ClosestUnexploredTargetSelector closestUnexploredTargetSelector;
	PandoraPathPlanner pandoraPathPlanner;
	//~ MapAttributes mapAttributes;
	
	//~ PartitionGraphNodes incrementalPartitionGraph; is now in partition_path_generator
	std::vector<PixelCoords> covPath;
	//~ std::vector<PixelCoords> goal;
	
	
	std::set<unsigned int> nodesQueue;			//!< Holds all the nodes' IDs of the voronoi graph
	std::vector<PixelCoords> path;				//!< Vector with path to target
	std::vector<PixelCoords> goal;				//!< Vector with subgoals
	//~ bool lockTarget;
	//~ unsigned char **coverage;	//!< The coverage matrix
	//~ PixelCoords *closestCoverageUnexplored;	//!< Thw closest unexplored point
	PixelCoords closest;	//!< Thw closest unexplored point
	PixelCoords prevTarget;	//!< Thw closest unexplored point
	//~ std::vector<PixelCoords> coverageLimits;	//!< Vector Coordinates of coverage limits
	
	
	//TODO: specify new operations. probably reinitialize voronoi voronoiNodes etc.
	void resetTargets(void);
	
	int selectNode(std::set<unsigned int> &graph);
	int selectNode_newWeights(std::set<unsigned int> &graph);
	
	//~ void calcNodeParameters(std::set<unsigned int> &graph);
	void calcNodeParameters(void);
	void calcNodeParameters_newWeights(void);
	
	double calcPathLength(std::vector <PixelCoords> &posPath);
	
	float calcAngleOfGoal(PixelCoords targetPos);
	
	void performNodesNormalization(std::set<unsigned int> &nodesIDs,VoronoiNodes &nodes);
	
	void performLinearNodesNormalization(std::set<unsigned int> &nodesIDs,VoronoiNodes &nodes);
	void performLinearNodesNormalization_newWeights(std::set<unsigned int> &nodesIDs,VoronoiNodes &nodes);
	
	
	float calcNodesWeight(int ID);
	float calcNodesWeight_newWeights(int ID);
	
	//needs editing
	//~ std::vector<PixelCoords> findCoverageLimits(unsigned char **map,MapInfo info,Transformation robotPose,int xMin,int xMax,int yMin,int yMax,int **field,unsigned char **coverage,PixelCoords *closest,int **addField);
	
	//~ bool checkPointIfCloseWall(unsigned char **map,MapInfo info,PixelCoords point,int xMin,int xMax,int yMin,int yMax,int **field);
	
	
	float calcEnclosure(PixelCoords p);
	
public:
	
	//TODO: create constructor. specify operations ( voronoi partitionGraph etc)
	ExplorationTargetSelector(MapAttributes& mapAttr, Coverage& cov);
	
	//~ PixelCoords selectTarget(void);
	void selectTarget(PixelCoords* target);	
	//~ void selectTarget(PixelCoords* target)
	
	
	void selectTargetWithGaussianExclusion(PixelCoords* target);
	
	void selectTargetWithGaussianExclusion_newWeights(PixelCoords* target);
	
};





