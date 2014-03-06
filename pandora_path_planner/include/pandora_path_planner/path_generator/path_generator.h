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

#ifndef PATH_GENERETOR_H
#define PATH_GENERETOR_H

#include "dijkstra.h"
#include "pandora_navigation_common/voronoi/voronoi.h"
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
