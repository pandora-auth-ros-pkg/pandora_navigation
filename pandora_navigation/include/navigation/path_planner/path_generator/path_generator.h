#ifndef PATH_GENERETOR_H
#define PATH_GENERETOR_H

#include "dijkstra.h"
#include "voronoi/voronoi.h"
#include "geometry_msgs/PoseStamped.h"
#include "vector"
//#include "map"
#include "path_generator_defines.h"


//class PathGenerator: public BaseGlobalPlanner{
class PathGenerator{
	
public:
	
	PathGenerator(MapAttributes* mapAttr,Voronoi * v);
	
	virtual bool generatePath(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)=0;
	
	
	
//protected:
	
	std::vector<PixelCoords> pixelPlan;				//!< Vector with path to target
	PixelCoords pixelGoal;							//!<Target
	PixelCoords robotPosition;
	//~ geometry_msgs::PoseStamped poseStampedGoal; //maybe not needed
	//~ std::vector<geometry_msgs::PoseStamped> poseStampedPlan; //maybe not needed
	
	
	//-------------------------------------------------------//
	
	MapAttributes* mapAttributes;
	
	Voronoi *voronoi;
	
	Dijkstra shortestPathFinder;
	
	PixelCoords transformPoseStamped2PixelCoords(geometry_msgs::PoseStamped point);
	geometry_msgs::PoseStamped transformPixelCoords2PoseStamped(PixelCoords point);
	
	
	//~ void minimizePath(PixelCoords startPos,PixelCoords target);
	void minimizePath(PixelCoords startPos, PixelCoords target, std::vector<PixelCoords>& wantedPath );
	
	bool checkIfValidPathMutation(std::vector<PixelCoords> &path, unsigned int index, PixelCoords startPos, PixelCoords target);
	
	double calcPathFitness(std::vector<PixelCoords> &posPath);
	
	double calcFitness(PixelCoords currPoint,PixelCoords prevPoint,PixelCoords nextPoint);
	
	bool isLineValid(PixelCoords p1,PixelCoords p2, unsigned char **map,float **brushCell,float op);
	//~ void resetTargets(void); // Η resetTarget χρησιμοποιείται και από την ΤargetSelector, μήπως να ενσωματωθεί και στις 2;
	
	
};


#endif
