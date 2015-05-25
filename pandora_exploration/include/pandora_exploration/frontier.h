#ifndef PANDORA_EXPLORATION_FRONTIER_H
#define PANDORA_EXPLORATION_FRONTIER_H

#include <boost/shared_ptr.hpp>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Header.h>

namespace pandora_exploration {

  /**
    * @class Frontier
    * @brief A class implementing the frontier object
    *
    * The class is needed for the representation of the frontier idea, used in
    * the frontier goal selector implementation.
    */
  class Frontier
  {
   public:
    
    /**
      * @brief Constructor for the Frontier class
      */
    Frontier() :
      size(0),
      min_distance(0.0),
      cost(0.0)
    { }
    
    /**
      * @brief Overload of operator "<"
      * @param other a frontier to compare
      * @return True if the frontier right to the operator has lower min distance
      * than the frontier to the left of the operator
      * 
      */
    bool operator< (const Frontier& other)
    {
      return (this->min_distance < other.min_distance);
    }

   public:

    std_msgs::Header header;

    uint32_t size;  // size of the frontier in cells
    float min_distance;  // frontier's minimum distance from robot using  bfs
    float cost;

    geometry_msgs::Point initial;  // initial point of the frontier
    geometry_msgs::Point centroid;  //?? frontier's centroid
    geometry_msgs::Point middle;  //?? the middle of the frontiers based on coordinates

    nav_msgs::Path path;  // the path to the frontier
  };

  typedef std::list<Frontier> FrontierList;
  typedef boost::shared_ptr<FrontierList> FrontierListPtr;

}  // namespace pandora_exploration

#endif  // PANDORA_EXPLORATION_FRONTIER_H
