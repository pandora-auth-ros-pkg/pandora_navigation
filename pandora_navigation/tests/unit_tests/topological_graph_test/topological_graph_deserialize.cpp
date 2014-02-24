#include <iostream>
#include "test/Image.h"
#include "map/map_attributes.h"
#include "highgui.h"
#include "cv.h"
#include <iostream>
#include "nodes/voronoi_nodes.h"
#include "test/test_utilities.h"
#include "test/serialization_IO.h"
#include <gtest/gtest.h>
#define MAP_SIZE 4096


TEST(TestArmMovement, moveServo) {
	
	MapAttributes mapAtt(MAP_SIZE,MAP_SIZE);

	initMapFromImage("map1.png",&mapAtt);

	
	int prevxMax = mapAtt.prevxMax ;
	int prevyMax = mapAtt.prevyMax ;
	int prevxMin = mapAtt.prevxMin ;
	int prevyMin = mapAtt.prevyMin ;


	Coverage coverage(mapAtt.getHeight(), mapAtt.getWidth());
	//-----------------------------------
	for(unsigned int i=0;i<30;i++){
		int x=0,y=0;
		while(mapAtt.map[x][y]<130){
			x=prevxMin+rand()%(prevxMax-prevxMin);
			y=prevyMin+rand()%(prevyMax-prevyMin);
		}
		makeDummyCoveragePatch(mapAtt.map,coverage.coverage,x,y,rand()%200);
	}
	//--------------------------------
	VoronoiNodes voronoiNodes(&mapAtt, &coverage);
	voronoiNodes.createVoronodes();

	VoronoiNodes ground_truth(&mapAtt , &coverage);
	restoreSerializedObject<VoronoiNodes>(ground_truth,"voronodes_marshall");

	//~ CvScalar pixel;
	
	//~ image mapSmall(prevxMax-prevxMin,prevyMax-prevyMin,"mapsmall",3);
	//~ for(unsigned int i=0;i<prevxMax-prevxMin;i++)
		//~ for(unsigned int j=0;j<prevyMax-prevyMin;j++){
			//~ pixel.val[0]=pixel.val[1]=pixel.val[2]=mapAtt.map[i+prevxMin][j+prevyMin];
			//~ if(coverage.coverage[i+prevxMin][j+prevyMin]>0)
				//~ pixel.val[0]=255-coverage.coverage[i+prevxMin][j+prevyMin];
			//~ mapSmall.setPoint(i,j,pixel);
			//~ if(voronoiNodes.voronoi.voronoi[i+prevxMin][j+prevyMin]){
				//~ pixel.val[0]=pixel.val[1]=0;
				//~ pixel.val[2]=255;
				//~ mapSmall.setPoint(i,j,pixel);
			//~ }
		//~ }
	//~ 
	//~ drawGraph(mapSmall.display,voronoiNodes.nodes,prevxMin,prevxMax,prevyMin,prevyMax);
	//~ mapSmall.saveImage("smallXartis.png");
	
	
	ASSERT_EQ( ground_truth.nodes.size() , voronoiNodes.nodes.size() );
	for(unsigned int i=0;i<voronoiNodes.nodes.size();i++){
		ASSERT_TRUE( ground_truth.nodes[i].p == voronoiNodes.nodes[i].p );
	}
	
}
	
int main(int argc, char **argv){
	
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();

}	
	
