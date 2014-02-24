#ifndef TREE_PATH_GENERATOR_H
#define TREE_PATH_GENERATOR_H

#include "navigation/path_planner/path_generator/path_generator.h"
#include "rrt_tree.h"
//~ #include "map/map_attributes.h" probably in voronoi


class TreePathGenerator: public PathGenerator{

public:
	
	//MapAttributes* mapAttributes;
	
	RRTTree robotTree;
	
	//~ Voronoi voronoi;
	
	
	//~ prevxMax,prevxMin,prevyMax,prevyMin,brushCell,
	
public:
	
	//~ void TreePathGenerator(PixelCoords startPosition,PixelCoords targetPosition);
	TreePathGenerator(MapAttributes* mapAttr,Voronoi *v);
	
	bool generatePath(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
	
	//~ void createSimpleRRT(PixelCoords startPos,PixelCoords target );
	void createSimpleRRT(PixelCoords startPos,PixelCoords target, std::vector<PixelCoords>& wantedPath);

};

#endif


