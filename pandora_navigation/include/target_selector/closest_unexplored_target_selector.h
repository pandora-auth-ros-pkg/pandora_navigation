#ifndef CLOSEST_UNEXPLORED_TARGET_SELECTOR_H
#define CLOSEST_UNEXPLORED_TARGET_SELECTOR_H


#include "target_selector.h"
#include "closet_unexplored_defines.h"
//#include "target_selector/closest_unexplored_defines.h"




class ClosestUnexploredTargetSelector: public TargetSelector{
	
public:
	
	//~ unsigned char** coverage;					//!< The coverage matrix
	//~ Coverage* coverage;
	//~ MapAttributes* mapAttributes;
	PixelCoords closestCoverageUnexplored;		//!< Thw closest unexplored point
	std::vector<PixelCoords> coverageLimits;	//!< Vector Coordinates of coverage limits
	int** addField;
	int** field;
	
	
	
	
public:

	//~ ClosestUnexploredTargetSelector(void);
	ClosestUnexploredTargetSelector(MapAttributes& mapAttr, Coverage& cov);
	
	void selectTarget(PixelCoords* target);
	
	//~ std::vector<PixelCoords> findCoverageLimits(unsigned char **map,MapInfo info,Transformation robotPose,int xMin,int xMax,int yMin,int yMax,int **field,unsigned char **coverage,PixelCoords *closest,int **addField);
	//std::vector<PixelCoords> findCoverageLimits(PixelCoords *closest);
	std::vector<PixelCoords> findCoverageLimits(unsigned char **map,int mapHeight,int mpaWidth,Transformation robotPose,int xMin,int xMax,int yMin,int yMax,int **field,unsigned char **coverage,PixelCoords *closest,int **addField);
	
	//~ bool checkPointIfCloseWall(unsigned char **map, MapInfo info, PixelCoords point, int xMin, int xMax, int yMin, int yMax, int **field);
	//bool checkPointIfCloseWall(PixelCoords point, int xMin, int xMax, int yMin, int yMax, int **field);
	bool checkPointIfCloseWall(unsigned char **map,int mapHeight, int mapWidth,PixelCoords point,int xMin,int xMax,int yMin,int yMax,int **field);
	
};


#endif


