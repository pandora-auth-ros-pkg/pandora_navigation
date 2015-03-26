#include <costmap_2d/costmap_2d.h>
#include <gtest/gtest.h>

class ExportCostmapTest : public ::testing::Test
  {
    protected:
      virtual void SetUp()
      {
        // Costmap Init data
        unsigned int sizeX = 100;
        unsigned int sizeY = 100;
        double resolution = 0.1;
        double originX = 0;
        double originY = 0;

        // Costmap creation
        costmapPtr.reset( new costmap_2d::Costmap2D(sizeX, sizeY, resolution, originX, originY) );

        // Fill costmap
        unsigned char* map = costmapPtr->getCharMap();
        const unsigned int size_x = costmapPtr->getSizeInCellsX(), size_y = costmapPtr->getSizeInCellsY();

        std::fill(map, map+ (size_x*size_y)/2, 150);  // fill the first half of the map with zeros
        std::fill(map+(size_x*size_y)/2 + 1, map+(size_x*size_y), 255);  //fill the second half with ones

        std::fill(map+(4000), map+(8000), 100);
        
        // Save costmap
        fp = costmapPtr->saveMap("/home/dimkirt/yolo.pgm");
        
        if(!fp)
        {
          std::cout<<" Costmap Export Failed! \n";
        }

      }

      // Accessor
      bool fileC()
      {
        return fp;
      }

      // Variables
      boost::shared_ptr<costmap_2d::Costmap2D> costmapPtr;
      bool fp;
  };

TEST_F(ExportCostmapTest, fileCreation)
{
  EXPECT_TRUE(fileC());
}

  
  
  

