/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, P.A.N.D.O.R.A. Team.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the P.A.N.D.O.R.A. Team nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Manos Tsardoulias
* Author: Aris Thallas
*********************************************************************/

#ifndef NAVIGATION_VORONOI_NODES_H
#define NAVIGATION_VORONOI_NODES_H


#include "nodes_vector.h"
#include "pandora_navigation_common/map/coverage.h"
#include "vector"
#include "map"
#include "utility"
#include "pandora_navigation_common/voronoi/voronoi.h"
#include "pandora_navigation_common/misc/identity.h"
#include "pandora_navigation_common/misc/connection.h"
#include "pandora_navigation_common/misc/possible_path.h"
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
