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

#ifndef PARTITION_GRAPH_NODES_DEFINES_H
#define PARTITION_GRAPH_NODES_DEFINES_H


//TODO:edit these

	#define WALL_DISTANCE		22   //!< The safe distance from wall in general
	#define CLOSE_TO_WALL 		WALL_DISTANCE  //!< The safe distance from wall in partition graph
	//~ #ifdef ONE_RRT					//!< Distance ,in various methods, that the robot should keep from the walls
		//~ #define WALL_DISTANCE		20   //!< The safe distance from wall in one RRT
	//~ #elif defined MULTIPLE_RRTS
		//~ #define WALL_DISTANCE		20   //!< The safe distance from wall in RRTs
	//~ #elif defined VISIBILITY_GRAPH
		//~ #define WALL_DISTANCE		25   //!< The safe distance from wall in Visibility Graph
	//~ #else
		
	//~ #endif
	//~ #define WALL_DISTANCE_SIMPLE_RRT 21  //!< The safe distance from wall in simple RRT


	
	//---D---/
	#define DECOMP_STEP			10								//!< Decomposision step lenght in pixels (the square side)
	#define DECOMP_STEP_DIAG	sqrt(2*pow(DECOMP_STEP,2))+1	//!< Decomposision square diagonal
	#define DECOMP_TARGET_DIST 	20								//!< Decomposision target and goal distance to make neighbors


#endif

