/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, P.A.N.D.O.R.A. Team.
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
* Author: Dimitrios Kirtsios <dimkirts@gmail.com>
*********************************************************************/
#include <costmap_2d/costmap_2d.h>
#include "pandora_exploration/map_frontier_search.h"
#include <iostream>
#include <gtest/gtest.h>

namespace pandora_exploration
{
  
  class MapFrontierSearchTest : public ::testing::Test
  {
    protected:
      
      virtual void SetUp()
      {
        ros::Time::init();  // Needed for ros::Time:now()

        // create a new costmap2D 10x10
        costmap_.reset( new costmap_2d::Costmap2D(1, 1, 0.1, 0.0, 0.0) );
        
        
        // Now pass the costmap created to mapFronterSearch 
        mapFrontierSearchPtr_.reset( new MapFrontierSearch(costmap_, "Camacho"));

      }
      
      /* HELPER FUNCTIONS */
      /**
       * Fills the column we pass as argument with the cost we pass.
       */
      void fillCostmapColumn(unsigned int col, unsigned char cost)
      {
        unsigned char* map = costmap_->getCharMap();
        unsigned int size_x = costmap_->getSizeInCellsX();
        unsigned int size_y = costmap_->getSizeInCellsY();

        for(unsigned int i=0; i<size_x; i++)
        {
           for(unsigned int j=0; j<size_y; j++)
           {
             if(col == j)
              costmap_->setCost(i,j,cost);
           } 
        }
      }

      /**
       * Fills the row we pass as argument with the cost we pass.
       */
      void fillCostmapRow(unsigned int col, unsigned char cost)
      {
        unsigned char* map = costmap_->getCharMap();
        unsigned int size_x = costmap_->getSizeInCellsX();
        unsigned int size_y = costmap_->getSizeInCellsY();

        for(unsigned int i=0; i<size_x; i++)
        {
           for(unsigned int j=0; j<size_y; j++)
           {
             if(col == i)
              costmap_->setCost(i,j,cost);
           } 
        }
      }

      /**
       * Prints the costmap in a way that is easy readable.
       */
      void printCostmap()
      {
        unsigned int i,j;
        unsigned int size_x = costmap_ -> getSizeInCellsX();
        unsigned int size_y = costmap_ -> getSizeInCellsY();

        std::cout<< "Printing costmap:" <<std::endl;
        
        std::cout<<"   ";
        for(j=0; j<size_y; j++)
        {
          std::cout<<" "<<j<<"  ";
        }
        std::cout<<std::endl;
        
        std::cout<<"   ";
        for(j=0; j<size_y; j++)
        {
          std::cout<<"--- ";
        }

        std::cout<<std::endl;
        for(i=0; i<size_x; i++)
        {
          for(j=0; j<size_y; j++)
          {
            if(j == 0)
            {
              std::cout<< i<<" |";
            }


            unsigned int cost = (int)costmap_->getCost(i,j);
            if(cost>99)
              std::cout<<cost<<" ";
            else if(cost>9)
              std::cout<<" "<<cost<<" ";
            else
              std::cout<<" "<<cost<<"  ";

          }
          std::cout<<std::endl;
        }
        
        std::cout<< "FREE_SPACE = 0" << std::endl;
        std::cout<< "LETHAL = 254" << std::endl;
        std::cout<< "UNKNOWN_SPACE = 255" << std::endl;
        std::cout<< std::endl;
      }


      /* MAP CREATORS */
      /**
       * Creates a costmap with a single frontier. Also prints the map.
       */
      void createMapSingleFrontier(const unsigned int size_x, const unsigned int size_y,
       const double resolution, const double origin_x, const double origin_y)
      {
        costmap_->resizeMap(size_x, size_y, resolution, origin_x, origin_y);
        unsigned char* map = costmap_->getCharMap();
        

        std::fill(map, map+ (size_x*size_y)/2 -1, 0);  // fill the first half of the map with zeros
        std::fill(map+(size_x*size_y)/2 , map+(size_x*size_y), 255);  //fill the second half with ones
      }
      
      

      /**
       * Creates a costmap with two frontiers. Also prints the map.
       */
      void createMapDoubleFrontier(const unsigned int size_x, const unsigned int size_y,
       const double resolution, const double origin_x, const double origin_y)
      {
        costmap_->resizeMap(size_x, size_y, resolution, origin_x, origin_y);
        unsigned char* map = costmap_->getCharMap();
        

        std::fill(map, map+ (size_x*size_y)/2 -1, 0);  // fill the first half of the map with zeros
        std::fill(map+(size_x*size_y)/2 , map+(size_x*size_y), 255);  //fill the second half with ones
        
        fillCostmapRow(3, 0);
      }


      /**
       * Creates a fully explored map (all zeros)
       */
      void createMapExplored(const unsigned int size_x, const unsigned int size_y,
       const double resolution, const double origin_x, const double origin_y)
      {
        costmap_->resizeMap(size_x, size_y, resolution, origin_x, origin_y);
        unsigned char* map = costmap_->getCharMap();
        

        std::fill(map, map+ (size_x*size_y), 0);  // fill costmap with zeros
        
      }

      /**
       * Creates a fully unknown map (all 255) EXTREME SCENARIO
       */
      void createMapUnknown(const unsigned int size_x, const unsigned int size_y,
       const double resolution, const double origin_x, const double origin_y)
      {
        costmap_->resizeMap(size_x, size_y, resolution, origin_x, origin_y);
        unsigned char* map = costmap_->getCharMap();
        

        std::fill(map, map+ (size_x*size_y), 255);  // fill costmap with 255
        
      }

      /**
       * Creates a fully unknown map (all 255) except a point near the middle of the map
       * EXTREME SCENARIO
       */
      void createMapKnownMidPoint(const unsigned int size_x, const unsigned int size_y,
       const double resolution, const double origin_x, const double origin_y)
      {
        costmap_->resizeMap(size_x, size_y, resolution, origin_x, origin_y);
        unsigned char* map = costmap_->getCharMap();
        
   
        std::fill(map, map+ (size_x*size_y), 255);  // fill costmap with 255

        // put middle known point on costmap
        std::fill(map + size_x/2 + (size_x*size_y)/2, map + 1 + size_x/2 + (size_x*size_y)/2, 0);
        
      }

      /**
       * Creates a fully known map (all 0) except a point near the middle of the map
       * EXTREME SCENARIO
       */
      void createMapUnknownMidPoint(const unsigned int size_x, const unsigned int size_y,
       const double resolution, const double origin_x, const double origin_y)
      {
        costmap_->resizeMap(size_x, size_y, resolution, origin_x, origin_y);
        unsigned char* map = costmap_->getCharMap();
        
        std::fill(map, map+ (size_x*size_y), 0);  // fill costmap with zeros

        // put middle unknown point on costmap
        std::fill(map + size_x/2 + (size_x*size_y)/2, map + 1 + size_x/2 + (size_x*size_y)/2, 255);
        
      }

      // Accessors
      Frontier buildNewFrontier(unsigned int initial_cell, unsigned int reference,
                                             std::vector<bool>& frontier_flag)
      {
        return mapFrontierSearchPtr_->buildNewFrontier(initial_cell, reference, frontier_flag);
      }

      bool isNewFrontierCell(unsigned int idx, const std::vector<bool>& frontier_flag)
      {
        return mapFrontierSearchPtr_->isNewFrontierCell(idx, frontier_flag);
      }

      // Given the coordinates returns the index
      unsigned int getIndex(unsigned int x, unsigned int y)
      {
        return costmap_->getIndex(x,y);
      }

      double getResolution()
      {
        return costmap_->getResolution();
      }

      void setCostmapCell(unsigned int x, unsigned int y, unsigned char cost)
      {
        costmap_ -> setCost(x, y, cost);
      }



      // Variables
      boost::shared_ptr<MapFrontierSearch> mapFrontierSearchPtr_;
      boost::shared_ptr<costmap_2d::Costmap2D> costmap_;

  };

  /**
   * There is no need for function isNewFrontierCell to be tested in different maps
   * because it's behaviour will be the same. It depends on the cell and its neighbours
   * not on the map. 
   */
  TEST_F(MapFrontierSearchTest, isNewFrontierCellTestFlagsFalse)
  {
    createMapDoubleFrontier(8, 8, 1.0, 0.0, 0.0);
    printCostmap();

    std::cout<< "Assuming no frontiers are marked (frontier flags = zero)" <<std::endl;
    
    // NORMAL CASE
    std::cout<< "Cell (2,4) IS frontier cell"<<std::endl;
    unsigned int idx = getIndex(2, 4);
    const std::vector<bool> flags (1, false);  //vector with 1 zero element

    EXPECT_TRUE(isNewFrontierCell(idx, flags));


    std::cout<< "Cell (3,4) IS NOT frontier cell"<<std::endl;
    idx = getIndex(3, 4);
    EXPECT_FALSE(isNewFrontierCell(idx, flags));

    std::cout<< "Cell (3,5) IS  NOT frontier cell"<<std::endl;
    idx = getIndex(3, 5);
    EXPECT_FALSE(isNewFrontierCell(idx, flags));

    // NO_INFO SPACE CASE
    std::cout<< "Cell (7,7) IS NOT frontier cell"<<std::endl;
    idx = getIndex(7, 7);
    EXPECT_FALSE(isNewFrontierCell(idx, flags));

    // KNOWN CELL CASE
    std::cout<< "Cell (4,4) IS  frontier cell"<<std::endl;
    idx = getIndex(4, 4);
    EXPECT_TRUE(isNewFrontierCell(idx, flags));

    std::cout<< "Cell (3,3) IS NOT frontier cell"<<std::endl;
    idx = getIndex(3, 3);
    EXPECT_FALSE(isNewFrontierCell(idx, flags));
  }

  /***** CASES WERE ALL FRONTIERS ARE MARKED  ********/
  TEST_F(MapFrontierSearchTest, isNewFrontierCellTestFlagsTrue)
  {
    createMapDoubleFrontier(8, 8, 1.0, 0.0, 0.0);
    printCostmap();

    
    std::cout<< "Assuming frontiers are MARKED (frontier flags = one)" <<std::endl;

     std::cout<< "Cell (2,4) IS frontier cell"<<std::endl;
    unsigned int idx = getIndex(2, 4);
    const std::vector<bool> flags (1, true);  

    EXPECT_FALSE(isNewFrontierCell(idx, flags));


    std::cout<< "Cell (3,4) IS NOT frontier cell"<<std::endl;
    idx = getIndex(3, 4);
    EXPECT_FALSE(isNewFrontierCell(idx, flags));

    std::cout<< "Cell (3,5) IS  NOT frontier cell"<<std::endl;
    idx = getIndex(3, 5);
    EXPECT_FALSE(isNewFrontierCell(idx, flags));

    // NO_INFO SPACE CASE
    std::cout<< "Cell (7,7) IS NOT frontier cell"<<std::endl;
    idx = getIndex(7, 7);
    EXPECT_FALSE(isNewFrontierCell(idx, flags));

    // KNOWN CELL CASE
    std::cout<< "Cell (4,4) IS  frontier cell"<<std::endl;
    idx = getIndex(4, 4);
    EXPECT_FALSE(isNewFrontierCell(idx, flags));

    std::cout<< "Cell (3,3) IS NOT frontier cell"<<std::endl;
    idx = getIndex(3, 3);
    EXPECT_FALSE(isNewFrontierCell(idx, flags));
  }




  /**
   * Testing buildNewFrontier Normal Case - Single Frontier
   */
  TEST_F(MapFrontierSearchTest, buildNewFrontierSingleFrontier)
  {
    std::cout<< "Building new frontier Test - Normal Case" << std::endl;
    // Map creation
    double res = 1.0;
    unsigned int sizeX = 8;
    unsigned int sizeY = 8;
    double originX = 0.0;  // origin of robot
    double originY = 0.0;
    
    // To build the frontier initial must be a cell with a neighbour in nhood8, who is frontier cell.
    unsigned int initialX = 0;  
    unsigned int initialY = 3;

    createMapSingleFrontier(sizeX, sizeY, res, originX, originY);
    setCostmapCell(originX, originY, 10);  // mark reference
    setCostmapCell(initialX, initialY, 66);  // mark initial cell
    fillCostmapColumn(5, 0);  // fill col 5 with zeros
    fillCostmapColumn(6, 0);
    printCostmap();
    

    unsigned int initial = getIndex(initialX, initialY);
    unsigned int reference = getIndex(originX, originY);  // robot pose
    
    std::vector<bool> flags (1, false);

    Frontier yolo = buildNewFrontier(initial, reference, flags);

    std::cout<< "Costmap resolution: "<< res <<std::endl;
    std::cout<< "Robot Origin: ("<<originX<<","<<originY<<")"<<std::endl;
    std::cout<< "Frontier cost: "<< yolo.cost <<std::endl;
    std::cout<< "Frontier min distance: "<< yolo.min_distance <<std::endl;
    std::cout<< "Frontier size: "<< yolo.size <<std::endl;
    // To size genika vgainei oso einai +1, akomh ki an den xtistei frontier dinei size 1.
    

    std::cout<< "Frontier initial: \n"<< yolo.initial <<std::endl;
    std::cout<< "Frontier centroid: \n"<< yolo.centroid <<std::endl;
    std::cout<< "Frontier middle: \n"<< yolo.middle <<std::endl;

    EXPECT_EQ(8, yolo.size);
    // initial frontier point
    EXPECT_NEAR(initialX, yolo.initial.x, 0.6);
    EXPECT_NEAR(initialY, yolo.initial.y, 0.6);
    // centroid frontier point
    // We can test the precision by looking the frontier
    EXPECT_NEAR(4, yolo.centroid.x, 1);
    EXPECT_NEAR(4, yolo.centroid.y, 1); // tolerance of one cell
    //EXPECT_NEAR();
    //EXPECT_NEAR();
    
    //EXPECT_EQ();
    //EXPECT_EQ();
  }
  

  /**
   * Testing buildNewFrontier Fully Explored
   */
   /*
  TEST_F(MapFrontierSearchTest, buildNewFrontierExploredMap)
  {
    std::cout<< "Building new frontier Test - Fully Explored Map" << std::endl;
    // Map creation
    double res = 1.0;
    unsigned int sizeX = 8;
    unsigned int sizeY = 8;
    double originX = 0.0;
    double originY = 0.0;
    
    // To build the frontier initial must be a cell with a neighbour in nhood8, who is frontier cell.
    unsigned int initialX = 3;  
    unsigned int initialY = 3;

    createMapExplored(sizeX, sizeY, res, originX, originY);
    //fillCostmapRow(4,0);
    printCostmap();
    

    unsigned int initial = getIndex(initialX, initialY);
    unsigned int reference = getIndex(originX, originY);  // robot pose
    
    std::vector<bool> flags (1, false);

    Frontier yolo = buildNewFrontier(initial, reference, flags);

    std::cout<< "Costmap resolution: "<< res <<std::endl;
    std::cout<< "Robot Origin: ("<<originX<<","<<originY<<")"<<std::endl;
    std::cout<< "Frontier cost: "<< yolo.cost <<std::endl;
    std::cout<< "Frontier min distance: "<< yolo.min_distance <<std::endl;
    std::cout<< "Frontier size: "<< yolo.size <<std::endl;
    // To size genika vgainei oso einai +1, akomh ki an den xtistei frontier dinei size 1.
    

    std::cout<< "Frontier initial: \n"<< yolo.initial <<std::endl;
    std::cout<< "Frontier centroid: \n"<< yolo.centroid <<std::endl;
    std::cout<< "Frontier middle: \n"<< yolo.middle <<std::endl;

    EXPECT_EQ(7, yolo.size);
  } */

  /**
   * Testing buildNewFrontier Unknown Map
   */
   /*
  TEST_F(MapFrontierSearchTest, buildNewFrontierUnknownMap)
  {
    std::cout<< "Building new frontier Test - Unknown Map" << std::endl;
    // Map creation
    double res = 1.0;
    unsigned int sizeX = 8;
    unsigned int sizeY = 8;
    double originX = 0.0;
    double originY = 0.0;
    
    // To build the frontier initial must be a cell with a neighbour in nhood8, who is frontier cell.
    unsigned int initialX = 3;  
    unsigned int initialY = 3;

    createMapUnknown(sizeX, sizeY, res, originX, originY);
    //fillCostmapRow(4,0);
    printCostmap();
    

    unsigned int initial = getIndex(initialX, initialY);
    unsigned int reference = getIndex(originX, originY);  // robot pose
    
    std::vector<bool> flags (1, false);

    Frontier yolo = buildNewFrontier(initial, reference, flags);

    std::cout<< "Costmap resolution: "<< res <<std::endl;
    std::cout<< "Robot Origin: ("<<originX<<","<<originY<<")"<<std::endl;
    std::cout<< "Frontier cost: "<< yolo.cost <<std::endl;
    std::cout<< "Frontier min distance: "<< yolo.min_distance <<std::endl;
    std::cout<< "Frontier size: "<< yolo.size <<std::endl;
    // To size genika vgainei oso einai +1, akomh ki an den xtistei frontier dinei size 1.
    

    std::cout<< "Frontier initial: \n"<< yolo.initial <<std::endl;
    std::cout<< "Frontier centroid: \n"<< yolo.centroid <<std::endl;
    std::cout<< "Frontier middle: \n"<< yolo.middle <<std::endl;

    EXPECT_EQ(7, yolo.size);
  } */
}  // namespace pandora_exploration