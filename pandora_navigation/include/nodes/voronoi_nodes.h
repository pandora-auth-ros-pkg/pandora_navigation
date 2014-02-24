#ifndef NAVIGATION_VORONOI_NODES_H
#define NAVIGATION_VORONOI_NODES_H


#include "nodes_vector.h"
#include "map/coverage.h"
#include "vector"
#include "map"
#include "utility"
#include "voronoi/voronoi.h"
#include "misc/identity.h"
#include "misc/connection.h"
#include "misc/possible_path.h"
#include "cstddef"
#include <boost/serialization/base_object.hpp>

//TODO: either fixVoronoi in constructor or add function to calculate voronoi

class VoronoiNodes: public NodesVector{
	
public:
	
	
	//~ VoronoiNodes(void);
	//VoronoiNodes(const MapAttributes* mapAttr, const Coverage cov);
	VoronoiNodes(MapAttributes* mapAttr, Coverage* cov); 
	
	
	void createVoronodes(void);
	
	
public:
	
	int** field;
	bool** isNode;
	
	//~ MapAttributes* mapAttributes; //in NodesVector class
	Coverage* coverage;
	Voronoi voronoi;
	
	//TODO: Are these needed here (if yes initialize in constructor)
	//~ bool** voronoi; 
	//~ unsigned char **map;		//!< The map matrix
	//~ unsigned char** coverage;	//!< The coverage matrix
	
	
	
	//~ void assignIDs(Voronodes &nodes);
	void assignIDs(void);
	
	
	//~ bool checkCrosspathNodes(unsigned int i,unsigned int j,bool **voronoi);
	bool checkCrosspathNodes(unsigned int i,unsigned int j);
	
	
	//~ bool checkForNeigbor(PixelCoords neigbor,std::vector<PixelCoords> &n);
	bool checkForNeigbor(PixelCoords neigbor,std::vector<PixelCoords> &n);
	
	
	//~ bool checkIfNodeInVoronodes(Voronode *n,Voronodes &nodes);
	bool checkIfNodeInVoronodes(Node *n);
	
	//~ void detectNodes(Voronodes &nodes,bool **voronoi,MapInfo info,int xMin,int xMax,int yMin,int yMax,unsigned 	char **coverage,bool **isNode,int **field,unsigned char ** map,Transformation robotPose);
	//void detectNodes(unsigned char** map, Transformation robotPose);
	void detectNodes(bool **voronoi, int mapHeight, int mapWidth,int xMin,int xMax,int yMin,int yMax, unsigned char **coverage,bool **isNode,int **field,unsigned char ** map,Transformation robotPose);
	
	
	//~ void eliminateNodes(Voronodes *nodes,unsigned char **coverage);
	void eliminateNodes(void);
	
	
	//~ void eliminateUnconnectedNodes(Voronodes &nodes);
	void eliminateUnconnectedNodes(void);
	
	
	//~ void eliminateUncoveredNodesPaths(Voronodes *nodes,unsigned char **coverage);
	void eliminateUncoveredNodesPaths(void);
	
	
	PixelCoords findCoordsFromId(unsigned int id); 
	
	
	//~ void getConnectedVoronoi(int **field,bool **voronoi,unsigned char **map,MapInfo info,Transformation robotPose, Voronodes &nodes,bool **isNode,int xMin,int xMax,int yMin,int yMax);
	void getConnectedVoronoi(int **field,bool **voronoi,unsigned char **map,int mapHeight, int mapWidth,Transformation robotPose, bool **isNode,int xMin,int xMax,int yMin,int yMax);
	//void getConnectedVoronoi(unsigned char** map, Transformation robotPose);
	
	
	//~ PixelCoords getOptClosest(int **brushCell,bool **voronoi, unsigned char **map, MapInfo info, PixelCoords position);
	//PixelCoords getOptClosest(unsigned char **map, PixelCoords position);
	PixelCoords getOptClosest(int **field,bool **voronoi, unsigned char **map, int mapHeight, int mapWidth, PixelCoords position);
	
	
	//~ void insertNodeNeighbors(Voronode *n,Voronodes &nodes,bool **isNode,bool **voronoi,MapInfo info,int xMin,int xMax,int yMin,int yMax,int **field,unsigned char **coverage);
	//void insertNodeNeighbors(Node *n);
	void insertNodeNeighbors(Node *n,bool **isNode,bool **voronoi, int mapHeight, int mapWidth,int xMin,int xMax,int yMin,int yMax,int **field,unsigned char **coverage);
	
	
	//~ void manipulateNodes(Voronodes &nodes,unsigned char **coverage);
	void manipulateNodes();
	
	
	//~ Voronode* transformcPixelCoordsToVoronoid(Voronodes *nodes,PixelCoords c){
	Node* transformPixelCoordsToNode(PixelCoords c);
	
	friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        // serialize base class information
        ar & boost::serialization::base_object<NodesVector>(*this);
        //~ ar & voronoi;
    }
	
};

#endif
