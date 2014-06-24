#ifndef FRONTIER_H_
#define FRONTIER_H_

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
    
    float proximity(const Frontier& other, std::string frontier_representation,
      double mergable_frontiers_deviation)
    {
      geometry_msgs::Point pose;
      geometry_msgs::Point other_pose;
      if (frontier_representation == "centroid") {
        pose = this->centroid;
        other_pose = other.centroid;
      }
      else if (frontier_representation == "middle") {
        pose = this->middle;
        other_pose = other.middle;
      }
      else {
        pose = this->initial;
        other_pose = other.initial;
      }
      float prox = exp( 
        -( 
          pow(pose.x - other_pose.x, 2.0) + 
          pow(pose.y - other_pose.y, 2.0)
        ) / pow( mergable_frontiers_deviation , 2.0 )
      );
      return prox;
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
  typedef std::list<Frontier>::iterator FrontierListIt;
  typedef boost::shared_ptr<FrontierList> FrontierListPtr;
  
} // namespace pandora_exploration

#endif
