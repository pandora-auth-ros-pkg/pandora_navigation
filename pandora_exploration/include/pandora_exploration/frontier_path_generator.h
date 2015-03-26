/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014 - 2015, P.A.N.D.O.R.A. Team.
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
* Author: Chris Zalidis <zalidis@gmail.com>,
          Dimitrios Kirtsios <dimkirts@gmail.com>
*********************************************************************/

#ifndef PANDORA_EXPLORATION_FRONTIER_PATH_GENERATOR_H
#define PANDORA_EXPLORATION_FRONTIER_PATH_GENERATOR_H

#include <string>
#include <nav_msgs/Path.h>

#include "pandora_exploration/frontier.h"

namespace pandora_exploration {

/**
  * @class FrontierPathGenerator
  * @brief Provides an interface for the frontier path generator used in frontier
  * goal selector. Every frontier path generator implementation must adhere 
  * to this interface. 
  */
class FrontierPathGenerator {
 public:
  
  /**
    * @brief Creates paths from start pose to a frontier, for all the frontiers we pass
    * @param start The start pose from where we create the path for each frontier
    * @param frontier_list A list with all the frontiers we want to generate paths for 
    * @return True when it has finished creating the paths for all the frontiers
    * inside frontier_list
    */
  virtual bool findPaths(const geometry_msgs::PoseStamped& start,
                         const FrontierListPtr& frontier_list) = 0;

  /**
    * @brief Destructor for the class FrontierPathGenerator
    */
  virtual ~FrontierPathGenerator()
  {
  }

 protected:
  
 /**
    * @brief Constructor for the class FrontierPathGenerator
    * @param name The name of the frontier path generator
    * @param frontier_represantation Set the frontier representation, centroid, middle or initial
    */
  FrontierPathGenerator(const std::string& name, const std::string& frontier_representation)
    : name_(name), frontier_representation_(frontier_representation)
  {
  }

 protected:
  // name of the frontier path generator
  std::string name_;

  // frontier representation, centroid, middle or initial
  std::string frontier_representation_;
};

typedef boost::shared_ptr<FrontierPathGenerator> FrontierPathGeneratorPtr;

}  // namespace pandora_exploration

#endif  // PANDORA_EXPLORATION_FRONTIER_PATH_GENERATOR_H
