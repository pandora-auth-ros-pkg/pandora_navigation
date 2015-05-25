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

#include <list>
#include <costmap_2d/costmap_2d.h>
#include <boost/foreach.hpp>
#include "pandora_exploration/costmap_tools.h"
#include <gtest/gtest.h>
namespace pandora_exploration
{
  class NeighbourFunctionTest : public ::testing::Test
  {
    protected:
      virtual void SetUp()
      {
        // create a new costmap2D 10x10
        // create costmap2D, sizeX,sizeY,res,originX,originY
        costmap_.reset( new costmap_2d::Costmap2D(10, 10, 0.1, 0.0, 0.0) );
      }

      // Variables
      boost::shared_ptr<costmap_2d::Costmap2D> costmap_;
  };

  // All possible cases of nhood4 and nhood8
  TEST_F(NeighbourFunctionTest, middle)
  {
    unsigned int index = costmap_ -> getIndex(4,4);  // 4,4 is a centric point
    
    ASSERT_EQ(4, nhood4(index, *costmap_).size());  // nhood returns a vector, so we check its size
    ASSERT_EQ(8, nhood8(index, *costmap_).size());
  }

  TEST_F(NeighbourFunctionTest, edge)
  {
    // Create a list of edge points to check the nhoods
    std::list<unsigned int> testEdgePointsList;
    testEdgePointsList.push_back(costmap_ -> getIndex(0,3));
    testEdgePointsList.push_back(costmap_ -> getIndex(0,7));
    testEdgePointsList.push_back(costmap_ -> getIndex(9,3));
    testEdgePointsList.push_back(costmap_ -> getIndex(9,7));
    testEdgePointsList.push_back(costmap_ -> getIndex(3,0));
    testEdgePointsList.push_back(costmap_ -> getIndex(7,9));
    
    BOOST_FOREACH(unsigned int index, testEdgePointsList)
    {
      ASSERT_EQ(3, nhood4(index, *costmap_).size());
      ASSERT_EQ(5, nhood8(index, *costmap_).size());
    }
  }

  
  TEST_F(NeighbourFunctionTest, corner)
  {
    // Create a list of all corner points to check the nhoods
    std::list<unsigned int> testEdgePointsList;
    testEdgePointsList.push_back(costmap_ -> getIndex(0,0));
    testEdgePointsList.push_back(costmap_ -> getIndex(0,9));
    testEdgePointsList.push_back(costmap_ -> getIndex(9,0));
    testEdgePointsList.push_back(costmap_ -> getIndex(9,9));
    
    BOOST_FOREACH(unsigned int index, testEdgePointsList)
    {
      ASSERT_EQ(2, nhood4(index, *costmap_).size());
      ASSERT_EQ(3, nhood8(index, *costmap_).size());
    }
    
  }

  TEST_F(NeighbourFunctionTest, offMap)
  {
    unsigned int index = costmap_ -> getIndex(15,15);
    ASSERT_EQ(0, nhood4(index, *costmap_).size());
    ASSERT_EQ(0, nhood8(index, *costmap_).size());
  }


  /* Nearest Cell Function Tests */

  class NearestCellTest : public ::testing::Test
  {
    protected:
      virtual void SetUp()
      { 
        // create a new costmap2D 10x10
        // create costmap2D, sizeX,sizeY,res,originX,originY
        costmap_.reset( new costmap_2d::Costmap2D(10, 10, 0.1, 0.0, 0.0) );

        unsigned char* map = costmap_ -> getCharMap();
        const unsigned int size_x = costmap_ -> getSizeInCellsX(), size_y = costmap_ -> getSizeInCellsY();

        std::fill(map, map+ (size_x*size_y)/2, 0);  // fill the first half of the map with zeros
        std::fill(map+(size_x*size_y)/2 + 1, map+(size_x*size_y), 100);  //fill the second half with ones
        
      }
      /* Prints the costmap */
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

      /* Prints the coordinates of the expected result and the real result */
      void printExpectationToReality(const unsigned int& expected_result,
       const unsigned int& real_result)
      {
        unsigned int x_expect, x_real, y_expect, y_real;
        costmap_ -> indexToCells(expected_result, x_expect, y_expect);
        costmap_ -> indexToCells(real_result, x_real, y_real);
        std::cout<< "Expected Result is: "<<x_expect<<", "<<y_expect<<std::endl;
        std::cout<< "Real Result is: "<<x_real<<", "<<y_real<<std::endl;
      }


      // Variables
      boost::shared_ptr<costmap_2d::Costmap2D> costmap_;      
  };

  //  TODO more tests, for bfs accuracy.
  /* Case where the value we want is at the starting cell */
  TEST_F(NearestCellTest, sameCell)
  {
    unsigned int input = costmap_ -> getIndex(0, 0);
    unsigned int result;  // result will hold the nearest cell with the value we want
    // bfs is used to decide the nearest cell.
    // Searches for value=1 starting from cell 95
    ASSERT_TRUE(pandora_exploration::nearestCell(result, input, 0, *costmap_));
    ASSERT_EQ(input, result);
  }
  
  /* Checks if nearestCell bfs is working */
  TEST_F(NearestCellTest, differentCell)
  {
    unsigned int input = costmap_ -> getIndex(0, 0);
    unsigned int result;
    ASSERT_TRUE(pandora_exploration::nearestCell(result, input, 100, *costmap_));
    ASSERT_NE(input, result);
  }
  
  /* Case where the cell is out off the map */
  TEST_F(NearestCellTest, offMap)
  {
    unsigned int input = std::numeric_limits<unsigned int>::max();
    unsigned int result;
    ASSERT_FALSE(pandora_exploration::nearestCell(result, input, 1, *costmap_));
  }

  /* Check if bfs works properly */
  TEST_F(NearestCellTest, bfsCheckFromCorner)
  {
    costmap_ -> setCost(9, 0, 13);  // mark the starting cell for visualization reasons
    printCostmap();
    unsigned int input = costmap_ -> getIndex(9, 0);
    unsigned int expected_result = costmap_ -> getIndex(9, 5);

    unsigned int real_result;
    ASSERT_TRUE(pandora_exploration::nearestCell(real_result, input, 100, *costmap_));
    ASSERT_EQ(expected_result, real_result);
    
    printExpectationToReality(expected_result, real_result);
  }


  TEST_F(NearestCellTest, bfsCheckFromCenter)
  {
    costmap_ -> setCost(4, 3, 13);
    printCostmap();
    unsigned int input = costmap_ -> getIndex(4, 3);
    unsigned int expected_result = costmap_ -> getIndex(4, 5);
    unsigned int real_result;
    ASSERT_TRUE(pandora_exploration::nearestCell(real_result, input, 100, *costmap_));
    ASSERT_EQ(expected_result, real_result);
    printExpectationToReality(expected_result, real_result);
  }

}  // namespace pandora_exploration
