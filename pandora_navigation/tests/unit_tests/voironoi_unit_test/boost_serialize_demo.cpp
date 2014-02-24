#include <cstddef> // NULL
#include <iomanip>
#include <iostream>
#include <fstream>
#include <string>

#include <boost/archive/tmpdir.hpp>

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/assume_abstract.hpp>



#include <iostream>
#include "Image.h"
#include "highgui.h"
#include "cv.h"
//~ #include "cxmisc.h"
//~ #include "cvaux.h"
#include <iostream>
//~ #include <opencv2/objdetect/objdetect.hpp>
//~ #include <opencv2/highgui/highgui.hpp>
//~ #include <opencv2/imgproc/imgproc.hpp>
#define MAP_SIZE 4096

#include "voronoi/voronoi.h"
#include "map/map_attributes.h"

using namespace std;
	


void saveVoronoi(const Voronoi &s, const char * filename){
    // make an archive
    std::ofstream ofs(filename);
    boost::archive::text_oarchive oa(ofs);
    oa << s;
}



void
restoreVoronoi(Voronoi &s, const char * filename)
{
    // open the archive
    std::ifstream ifs(filename);
    boost::archive::text_iarchive ia(ifs);

    // restore the schedule from the archive
    ia >> s;
}



int main(){
	
	std::string filePath="map1.png";
	unsigned char ** map;
	image mapImg(filePath.c_str());
	map=new unsigned char *[MAP_SIZE];
	for(unsigned int i=0;i<MAP_SIZE;i++)
		map[i]=new unsigned char [MAP_SIZE];
	mapImg.writeDataToPointer(map);

	int prevxMax,prevyMax,prevxMin,prevyMin;
	prevxMax=prevyMax=4096;
	prevxMin=prevyMin=100;
	bool done;
	if(prevxMin<0) prevxMin=100;
	if(prevyMin<0) prevyMin=100;
	
	//	Check for prevxMin

	done=false;
	prevxMin=MAP_SIZE/2;
	while(!done){
		done=true;
		for(int i=prevyMin;i<prevyMax;i++){
			if(map[prevxMin][i]!=127){
				prevxMin-=64;
				done=false;
				break;
			}
		}
	}

	//	Check for prevxMax
	done=false;
	prevxMax=MAP_SIZE/2;
	while(!done){
		done=true;
		for(int i=prevyMin;i<prevyMax;i++){
			if(map[prevxMax][i]!=127){
				prevxMax+=64;
				done=false;
				break;
			}
		}
		
	}

	//	Check for prevyMin

	done=false;
	prevyMin=MAP_SIZE/2;
	while(!done){
		done=true;
		for(int i=prevxMin;i<prevxMax;i++){
			if(map[i][prevyMin]!=127){
				prevyMin-=64;
				done=false;
				break;
			}
		}

	}

	//	Check for prevyMax

	done=false;
	prevyMax=MAP_SIZE/2;
	while(!done){
		done=true;
		for(int i=prevxMin;i<prevxMax;i++){
			if(map[i][prevyMax]!=127){
				done=false;
				prevyMax+=64;
				break;
			}
		}
	}

	MapAttributes mapAtt(MAP_SIZE,MAP_SIZE);
	mapAtt.prevxMax=prevxMax;
	mapAtt.prevxMin=prevxMin;
	mapAtt.prevyMax=prevyMax;
	mapAtt.prevyMin=prevyMin;
	
	mapAtt.map=map;
	Voronoi voronoi(&mapAtt , false);
	voronoi.fixVoronoi();
	
	saveVoronoi(voronoi,"voronoi_marshall");


	image voronoiImg(prevxMax-prevxMin,prevyMax-prevyMin,"voronoi",1);
	CvScalar pixel;
	pixel.val[0]=255;
	
	image mapSmall(prevxMax-prevxMin,prevyMax-prevyMin,"mapsmall",3);
	for(unsigned int i=0;i<prevxMax-prevxMin;i++)
		for(unsigned int j=0;j<prevyMax-prevyMin;j++){
			pixel.val[0]=pixel.val[1]=pixel.val[2]=map[i+prevxMin][j+prevyMin];
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
	

