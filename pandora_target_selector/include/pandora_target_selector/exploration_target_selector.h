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

#include "closest_unexplored_target_selector.h"
#include "pandora_navigation_common/nodes/voronoi_nodes.h"
#include "pandora_path_planner/pandora_path_planner.h"
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





