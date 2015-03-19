#include <costmap_2d/cost_values.h>
#include <boost/foreach.hpp>

#include "pandora_exploration/costmap_tools.h"
#include "pandora_exploration/map_frontier_search.h"

namespace pandora_exploration {

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
using costmap_2d::FREE_SPACE;

MapFrontierSearch::MapFrontierSearch(const boost::shared_ptr<costmap_2d::Costmap2D>& costmap,
                                     std::string costmap_frame)
  : FrontierSearch(costmap, costmap_frame)
{
}

std::list<Frontier> MapFrontierSearch::searchFrom(geometry_msgs::Point position)
{

  std::list<Frontier> frontier_list;

  // make sure map is consistent and locked for duration of search
  boost::unique_lock<boost::shared_mutex> lock(*(costmap_->getLock()));

  // Sanity check that robot is inside costmap bounds before searching
  unsigned int mx, my;
  if (!costmap_->worldToMap(position.x, position.y, mx, my)) {
    ROS_ERROR("Robot out of costmap bounds, cannot search for frontiers");
    return frontier_list;
  }

  unsigned char* map = costmap_->getCharMap();
  unsigned int size_x = costmap_->getSizeInCellsX();
  unsigned int size_y = costmap_->getSizeInCellsY();

  // initialize flag arrays to keep track of visited and frontier cells
  std::vector<bool> frontier_flag(size_x * size_y, false);
  std::vector<bool> visited_flag(size_x * size_y, false);

  // initialize breadth first search
  std::queue<unsigned int> bfs;

  // find closest clear cell to start search
  unsigned int clear, pos = costmap_->getIndex(mx, my);
  if (nearestCell(clear, pos, FREE_SPACE, *costmap_)) {
    bfs.push(clear);
  } else {
    bfs.push(pos);
    ROS_WARN("Could not find nearby clear cell to start search");
  }
  visited_flag[bfs.front()] = true;

  while (!bfs.empty()) {
    unsigned int idx = bfs.front();
    bfs.pop();

    // iterate over 4-connected neighbourhood
    BOOST_FOREACH(unsigned nbr, nhood4(idx, *costmap_))
    {
      // add to queue all free, unvisited cells, use descending search in case initialized on
      // non-free cell
      if (map[nbr] <= map[idx] && !visited_flag[nbr]) {
        visited_flag[nbr] = true;
        bfs.push(nbr);
        // check if cell is new frontier cell (unvisited, NO_INFORMATION, free neighbour)
      } else if (isNewFrontierCell(nbr, frontier_flag)) {
        frontier_flag[nbr] = true;
        Frontier new_frontier = buildNewFrontier(nbr, pos, frontier_flag);
        if (new_frontier.size > 1) {
          frontier_list.push_back(new_frontier);
        }
      }
    }
  }

  return frontier_list;
}

Frontier MapFrontierSearch::buildNewFrontier(unsigned int initial_cell, unsigned int reference,
                                             std::vector<bool>& frontier_flag)
{

  // initialize frontier structure
  Frontier output;
  output.centroid.x = 0;
  output.centroid.y = 0;
  output.size = 1;
  output.min_distance = std::numeric_limits<double>::infinity();

  // record initial contact point for frontier
  unsigned int ix, iy;
  costmap_->indexToCells(initial_cell, ix, iy);
  costmap_->mapToWorld(ix, iy, output.initial.x, output.initial.y);

  // push initial gridcell onto queue
  std::queue<unsigned int> bfs;
  bfs.push(initial_cell);

  // cache reference position in world coords
  unsigned int rx, ry;
  double reference_x, reference_y;
  costmap_->indexToCells(reference, rx, ry);
  costmap_->mapToWorld(rx, ry, reference_x, reference_y);

  while (!bfs.empty()) {
    unsigned int idx = bfs.front();
    bfs.pop();

    // try adding cells in 8-connected neighborhood to frontier
    BOOST_FOREACH(unsigned int nbr, nhood8(idx, *costmap_))
    {
      // check if neighbour is a potential frontier cell
      if (isNewFrontierCell(nbr, frontier_flag)) {

        // mark cell as frontier
        frontier_flag[nbr] = true;
        unsigned int mx, my;
        double wx, wy;
        costmap_->indexToCells(nbr, mx, my);
        costmap_->mapToWorld(mx, my, wx, wy);

        // update frontier size
        output.size++;

        // update centroid of frontier
        output.centroid.x += wx;
        output.centroid.y += wy;

        // determine frontier's distance from robot, going by closest gridcell to robot
        double distance =
            sqrt(pow((static_cast<double>(reference_x) - static_cast<double>(wx)), 2.0) +
                 pow((static_cast<double>(reference_y) - static_cast<double>(wy)), 2.0));
        if (distance < output.min_distance) {
          output.min_distance = distance;
          output.middle.x = wx;
          output.middle.y = wy;
        }

        // add to queue for breadth first search
        bfs.push(nbr);
      }
    }
  }

  // average out frontier centroid
  output.centroid.x /= output.size;
  output.centroid.y /= output.size;

  // keep track of creation time
  output.header.stamp = ros::Time::now();
  output.header.frame_id = costmap_frame_;
  //~     output.header.frame_id = "map";
  return output;
}

bool MapFrontierSearch::isNewFrontierCell(unsigned int idx, const std::vector<bool>& frontier_flag)
{

  unsigned char* map = costmap_->getCharMap();

  // check that cell is unknown and not already marked as frontier
  if (map[idx] != NO_INFORMATION || frontier_flag[idx]) {
    return false;
  }

  // frontier cells should have at least one cell in 4-connected neighbourhood that is free
  BOOST_FOREACH(unsigned int nbr, nhood4(idx, *costmap_))
  {
    if (map[nbr] == FREE_SPACE) {
      return true;
    }
  }

  return false;
}

}  // namespace pandora_exploration
