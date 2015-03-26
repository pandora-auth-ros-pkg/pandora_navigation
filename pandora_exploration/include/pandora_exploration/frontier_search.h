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

#ifndef PANDORA_EXPLORATION_FRONTIER_SEARCH_H
#define PANDORA_EXPLORATION_FRONTIER_SEARCH_H

#include <string>
#include <list>
#include <boost/shared_ptr.hpp>

#include <geometry_msgs/Point.h>
#include <costmap_2d/costmap_2d.h>

#include "pandora_exploration/frontier.h"

namespace pandora_exploration {

/**
  * @class FrontierSearch
  * @brief Provides an interface for the frontier searcher used in frontier goal selector.
  * Every frontier searcher implementation must adhere to this interface. 
  */
class FrontierSearch {
 public:
  virtual std::list<Frontier> searchFrom(geometry_msgs::Point position) = 0;

  /**
    * @brief Destructor for the class FrontierSearch
    */
  virtual ~FrontierSearch()
  {
  }

 protected:
  /**
    * @brief Constructor for the class FrontierSearch
    * @param costmap A ptr to a costmap
    * @param costmap_frame The frame of the costmap
    */
  FrontierSearch(const boost::shared_ptr<costmap_2d::Costmap2D>& costmap, std::string costmap_frame)
    : costmap_(costmap), costmap_frame_(costmap_frame)
  {
  }

 protected:
  
  // shared ptr that holds the costmap
  boost::shared_ptr<costmap_2d::Costmap2D> costmap_;
  std::string costmap_frame_;
};

typedef boost::shared_ptr<FrontierSearch> FrontierSearchPtr;

}  // namespace pandora_exploration

#endif  // PANDORA_EXPLORATION_FRONTIER_SEARCH_H
