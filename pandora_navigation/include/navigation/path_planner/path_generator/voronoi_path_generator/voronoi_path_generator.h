#ifndef VORONOI_PATH_GENERATOR_H
#define VORONOI_PATH_GENERATOR_H

#include "navigation/path_planner/path_generator/path_generator.h"
#include "voronoi_path_generator_defines.h"

class VoronoiPathGenerator: public PathGenerator{
	
private:
	
	//~ Voronoi voronoi;
	int** field;

	PixelCoords getOptClosest(int **brushCell,bool **voronoi, unsigned char **map, int mapHeight, int mapWigth, PixelCoords position);
	
	//~ std::vector<PixelCoords> findOptPathToPoint(int **field,bool **voronoi, unsigned char **map, MapInfo info, PixelCoords position, PixelCoords destination);
	//probably needs map and map info removed
	std::vector<PixelCoords> findOptPathToPoint( bool** voronoi, unsigned char **map,int mapHeight, int mapWigth, PixelCoords position, PixelCoords destination);
	
	void drawOptField(bool **voronoi, int **field,int mapHeight, int mapWigth, PixelCoords position, PixelCoords destination);
	
	//~ void moveWithVoronoi(PixelCoords startPos, PixelCoords target, std::vector<PixelCoords>& wantedPath );
	//void moveWithVoronoi(PixelCoords startPos, PixelCoords target, std::vector<PixelCoords>& wantedPath, bool **voronoi );
	void moveWithVoronoi(PixelCoords startPos, PixelCoords target, std::vector<PixelCoords>& wantedPath );
	
public:
	
	//~ void VoronoiPathGenerator(void);
	VoronoiPathGenerator(MapAttributes* mapAttr,Voronoi *v);
	
	bool generatePath(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
	
	
};


#endif


