#include "map/map_attributes.h"

MapAttributes::MapAttributes(int height, int width){
	
	this->height = height;
	this->width = width;
	
	map=new unsigned char *[height];
	for(unsigned int i=0;i<height;i++)
		map[i]=new unsigned char [width];
	
	for(unsigned int i=0;i<height;i++)
		for(unsigned int j=0;j<width;j++)
			map[i][j]=127;
	
	robotTrajectory.clear();
	
	xMax=xMin=yMax=yMin=4096/2;
}

