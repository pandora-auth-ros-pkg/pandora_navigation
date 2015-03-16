#ifndef PANDORA_EXPLORATION_FRONTIER_H
#define PANDORA_EXPLORATION_FRONTIER_H

#include <boost/shared_ptr.hpp>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Header.h>

namespace pandora_exploration {

  class Frontier
  {
   public:
    Frontier() :
      size(0),
      min_distance(0.0),
      cost(0.0)
    { }

    bool operator< (const Frontier& other)
    {
      return (this->min_distance < other.min_distance);
    }

   public:

    std_msgs::Header header;

    uint32_t size;
    //min distance using bfs
    float min_distance;
    float cost;

    geometry_msgs::Point initial;
    geometry_msgs::Point centroid;
    geometry_msgs::Point middle;

    nav_msgs::Path path;
  };

  typedef std::list<Frontier> FrontierList;
  typedef boost::shared_ptr<FrontierList> FrontierListPtr;

} // namespace pandora_exploration

#endif  // PANDORA_EXPLORATION_FRONTIER_H
