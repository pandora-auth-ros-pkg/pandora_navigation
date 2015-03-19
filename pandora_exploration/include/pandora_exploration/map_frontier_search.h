#ifndef PANDORA_EXPLORATION_MAP_FRONTIER_SEARCH_H
#define PANDORA_EXPLORATION_MAP_FRONTIER_SEARCH_H

#include "pandora_exploration/frontier_search.h"

namespace pandora_exploration {

/**
 * @brief Thread-safe implementation of a frontier-search task for an input costmap.
 */
class MapFrontierSearch : public FrontierSearch {

 public:
  /**
   * @brief Constructor for search task
   * @param costmap Reference to costmap data to search.
   */
  MapFrontierSearch(const boost::shared_ptr<costmap_2d::Costmap2D>& costmap,
                    std::string costmap_frame);

  /**
   * @brief Runs search implementation, outward from the start position
   * @param position Initial position to search from
   * @return List of frontiers, if any
   */
  virtual std::list<Frontier> searchFrom(geometry_msgs::Point position);

  ~MapFrontierSearch()
  {
  }

 private:
  /**
   * @brief Starting from an initial cell, build a frontier from valid adjacent cells
   * @param initial_cell Index of cell to start frontier building
   * @param reference Reference index to calculate position from
   * @param frontier_flag Flag vector indicating which cells are already marked as frontiers
   * @return
   */
  Frontier buildNewFrontier(unsigned int initial_cell, unsigned int reference,
                            std::vector<bool>& frontier_flag);

  /**
   * @brief isNewFrontierCell Evaluate if candidate cell is a valid candidate for a new frontier.
   * @param idx Index of candidate cell
   * @param frontier_flag Flag vector indicating which cells are already marked as frontiers
   * @return
   */
  bool isNewFrontierCell(unsigned int idx, const std::vector<bool>& frontier_flag);
};

}  // namespace pandora_exploration
#endif  // PANDORA_EXPLORATION_MAP_FRONTIER_SEARCH_H
