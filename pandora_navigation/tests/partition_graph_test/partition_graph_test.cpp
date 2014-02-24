#include <iostream>

#include "map/map_attributes.h"
#include "highgui.h"
#include "nodes/partition_graph_nodes.h"
#include "cv.h"
#include <iostream>
#include "test/test_utilities.h"
#define MAP_SIZE 4096

#define d_PI_DOUBLE     		6.283185308


int main(){

	MapAttributes mapAtt(MAP_SIZE,MAP_SIZE);

	initMapFromImage("map1.png",&mapAtt);

		
	int prevxMax = mapAtt.prevxMax ;
	int prevyMax = mapAtt.prevyMax ;
	int prevxMin = mapAtt.prevxMin ;
	int prevyMin = mapAtt.prevyMin ;


//~ std::cout<<"after map"<<"\n";
	//~ Coverage coverage(mapAtt.getHeight(), mapAtt.getWidth());
	//~ //-----------------------------------
	//~ for(unsigned int i=0;i<30;i++){
		//~ int x=0,y=0;
		//~ while(map[x][y]<130){
			//~ x=prevxMin+rand()%(prevxMax-prevxMin);
			//~ y=prevyMin+rand()%(prevyMax-prevyMin);
		//~ }
		//~ makeDummyCoveragePatch(map,coverage.coverage,x,y,rand()%200);
	//~ }
	//~ //--------------------------------
	//~ VoronoiNodes voronoiNodes(&mapAtt, &coverage);
	//~ voronoiNodes.createVoronodes();
	
	
	
	Voronoi voronoi (&mapAtt,false);
	
	PixelCoords pose = PixelCoords(mapAtt.robotPose.dx, mapAtt.robotPose.dy);
	
	
	//~ std::cout<<"pre graph definition"<<"\n";
	PartitionGraphNodes partitionGraph(&mapAtt,&voronoi);
	//~ std::cout<<"ante graph definition"<<"\n";
	partitionGraph.createIncrementalPartition(pose,pose);
	
	
	

	
	 
	
	CvScalar pixel;
	
	image mapSmall(prevxMax-prevxMin,prevyMax-prevyMin,"mapsmall",3);
	for(unsigned int i=0;i<prevxMax-prevxMin;i++)
		for(unsigned int j=0;j<prevyMax-prevyMin;j++){
			pixel.val[0]=pixel.val[1]=pixel.val[2]=mapAtt.map[i+prevxMin][j+prevyMin];
			//~ if(coverage.coverage[i+prevxMin][j+prevyMin]>0)
				//~ pixel.val[0]=255-coverage.coverage[i+prevxMin][j+prevyMin];
			mapSmall.setPoint(i,j,pixel);
			//~ if(voronoiNodes.voronoi.voronoi[i+prevxMin][j+prevyMin]){
				//~ pixel.val[0]=pixel.val[1]=0;
				//~ pixel.val[2]=255;
				//~ mapSmall.setPoint(i,j,pixel);
			//~ }
		}
	
	drawGraph(mapSmall.display,partitionGraph.nodes,prevxMin,prevxMax,prevyMin,prevyMax);
	mapSmall.saveImage("smallXartis.png");
	mapSmall.show(0);
}
	
	
	
