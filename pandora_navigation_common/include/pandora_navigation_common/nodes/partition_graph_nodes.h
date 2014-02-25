#ifndef PARTITION_GRAPG_NODES_H
#define PARTITION_GRAPG_NODES_H

#include "nodes_vector.h"
#include "pandora_navigation_common/voronoi/voronoi.h"
#include "vector"
#include "ros/ros.h"
#include "pandora_navigation_common/misc/partition_graph_nodes_defines.h"

class PartitionGraphNodes: public NodesVector{
	
private:
	
	//~ float** brushCell;
	Voronoi* voronoi;
	
	int** decomp;		//!< The nodes matrix
	
	std::vector<int> targetNeighbors;			//!<Vector holding target neighbors
	
	std::vector<int> robotNeighbors;			//!<Vector holding robot neighbors
	
	std::vector<PixelCoords> uniformPartition;
	
	//~ void performIncrementalPartition(vector <PixelCoords> &uniformPartition, int xmax, int xmin,int ymax, int ymin,unsigned char **map,float **brushCell,int **decomp);
	//~ void performIncrementalPartition(unsigned char **map);
	void performIncrementalPartition( int xmax, int xmin,int ymax, int ymin,unsigned char **map,float **brushCell );
	
	void reinitialiseIncrementalPartition(void);
	
public:
	
	//~ partitionGraphNodes(void);
	PartitionGraphNodes(MapAttributes* mapAttr, Voronoi* vor);
	
	
	void createIncrementalPartition(PixelCoords startPos, PixelCoords target);
	
	//~ TODO:possibly edit to work with Node as argument
	void eliminateNode(int ID,bool keepPosition);
	
	int insertNodeInPartition(Node currNode);

	
	// Η computeDistanceFrom που θα μπει πλέον;
};


#endif
