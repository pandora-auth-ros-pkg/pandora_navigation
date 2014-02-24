#include <iostream>
#include "test/Image.h"
#include "map/map_attributes.h"
#include "highgui.h"
#include "cv.h"
#include "voronoi/voronoi.h"
#include "test/test_utilities.h"
#define MAP_SIZE 4096

int main(){
	
	MapAttributes mapAtt(MAP_SIZE,MAP_SIZE);

	initMapFromImage("map1.png",&mapAtt);
	
	Voronoi voronoi(&mapAtt , false);
	voronoi.fixVoronoi();


	int prevxMax = mapAtt.prevxMax ;
	int prevyMax = mapAtt.prevyMax ;
	int prevxMin = mapAtt.prevxMin ;
	int prevyMin = mapAtt.prevyMin ;

	image voronoiImg(prevxMax-prevxMin,prevyMax-prevyMin,"voronoi",1);
	CvScalar pixel;
	pixel.val[0]=255;
	
	image mapSmall(prevxMax-prevxMin,prevyMax-prevyMin,"mapsmall",3);
	for(unsigned int i=0;i<prevxMax-prevxMin;i++)
		for(unsigned int j=0;j<prevyMax-prevyMin;j++){
			pixel.val[0]=pixel.val[1]=pixel.val[2]=mapAtt.map[i+prevxMin][j+prevyMin];
			mapSmall.setPoint(i,j,pixel);
			if(voronoi.voronoi[i+prevxMin][j+prevyMin]){
				pixel.val[0]=pixel.val[1]=0;
				pixel.val[2]=255;
				mapSmall.setPoint(i,j,pixel);
			}
		}
	
	for(unsigned int i=0;i<prevxMax-prevxMin;i++)
		for(unsigned int j=0;j<prevyMax-prevyMin;j++)
			if(voronoi.voronoi[i+prevxMin][j+prevyMin])
				voronoiImg.setPoint(i,j,pixel);
	
	mapSmall.saveImage("smallXartis.png");
	voronoiImg.saveImage("voronoiasmenosXartis.png");
	mapSmall.show(0);
}
	
	
	
