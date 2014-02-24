#include <iostream>
#include "test/Image.h"
#include "map/map_attributes.h"
#include "highgui.h"
#include "cv.h"
#include "target_selector/exploration_target_selector.h"
//~ #include "cxmisc.h"
//~ #include "cvaux.h"
#include <iostream>
#include "misc/pixelcoords.h"
//~ #include <opencv2/objdetect/objdetect.hpp>
//~ #include <opencv2/highgui/highgui.hpp>
//~ #include <opencv2/imgproc/imgproc.hpp>
//~ #include "voronoi.h"
//~ #include "voronoi_nodes.h"
#define MAP_SIZE 4096

#define d_PI_DOUBLE     		6.283185308

using namespace std;

std::vector<PixelCoords> getLine(PixelCoords p1,PixelCoords p2){
	vector<PixelCoords> ret;
	ret.clear();
	int x1,x2,y1,y2,temp;
	x1=p1.getXCoord();
	x2=p2.getXCoord();
	y1=p1.getYCoord();
	y2=p2.getYCoord();
	if(x1==x2){
		if(y1>y2){
			temp=y1;
			y1=y2;
			y2=temp;
		}
		for(int i=y1;i<y2;i++)
			ret.push_back(PixelCoords(x1,i));
		return ret;
	}
	if(x1>x2){
		temp=x1;
		x1=x2;
		x2=temp;
		temp=y1;
		y1=y2;
		y2=temp;
	}
	float l=((float)(y2-(float)y1)/(float)(x2-(float)x1));
	if((x2-x1)>abs(y2-y1)){	//Must find y's
		for(int i=x1;i<x2;i++)
			ret.push_back(PixelCoords(i,(int(l*i-l*x1+(float)y1))));
		return ret;
	}
	else{					//Must find x's
		if(y1>y2){
			temp=x1;
			x1=x2;
			x2=temp;
			temp=y1;
			y1=y2;
			y2=temp;
		}
		for(int i=y1;i<y2;i++)
			ret.push_back(PixelCoords((i-(float)y1+l*x1)/l,i));
			
			
		return ret;
	}
}


void makeDummyCoveragePatch(unsigned char **map,unsigned char **coverage,int x, int y,int d){
	float angle=0;
	int tx,ty;
	while (angle<d_PI_DOUBLE){
		for(unsigned int i=0;i<d;i++){
			tx=x+i*cos(angle);
			ty=y+i*sin(angle);
			if((coverage[tx][ty]+3)<=255 && map[tx][ty]>127)
				coverage[tx][ty]+=3;
		}
		angle+=0.0004;
	}
}
void drawCross(IplImage *mapDisplay,int x,int y, int size,CvScalar pixel,std::string func){
	if((x-size)<0 || (y-size)<0 || (x+size)>=mapDisplay->width || (y+size)>=mapDisplay->height){
		return;
	}
	cvSet2D(mapDisplay,y,x,pixel);
	for( int i=1;i<=size;i++){
		cvSet2D(mapDisplay,y+i,x,pixel);
		cvSet2D(mapDisplay,y-i,x,pixel);
		cvSet2D(mapDisplay,y,x+i,pixel);
		cvSet2D(mapDisplay,y,x-i,pixel);
	}
}
void drawGraph(IplImage *mapDisplay,std::vector<Node> nodes,int xMin,int xMax,int yMin,int yMax){
	//	Then draw the connections
		for(unsigned int i=0;i<nodes.size();i++){
			for(unsigned int j=0;j<nodes[i].neigh.size();j++){
				vector<PixelCoords> tempv=getLine(nodes[i].p,nodes[i].neigh[j]);
				for(unsigned int k=0;k<tempv.size();k++){
					drawCross(mapDisplay,tempv[k].getXCoord()-xMin,yMax-tempv[k].getYCoord(),1,cvScalar(46,139,87),"");
				}
			}
		}
	//~ //	First draw the nodes		

		for(unsigned int i=0;i<nodes.size();i++){

			int x1=nodes[i].p.getXCoord()-xMin-5;
			int y1=yMax-nodes[i].p.getYCoord()-5;
			int x2=nodes[i].p.getXCoord()-xMin+5;
			int y2=yMax-nodes[i].p.getYCoord()-5;
			int x3=nodes[i].p.getXCoord()-xMin+5;
			int y3=yMax-nodes[i].p.getYCoord()+5;
			int x4=nodes[i].p.getXCoord()-xMin-5;
			int y4=yMax-nodes[i].p.getYCoord()+5;
			CvPoint  curve1[]={x1,y1, x2,y2, x3,y3, x4,y4};
			CvPoint* curveArr[1]={curve1};
			int      nCurvePts[1]={4};
			int      nCurves=1;
			int      isCurveClosed=1;
			int      lineWidth=1;
			
			if(nodes[i].neigh.size()==1){
				cvPolyLine(mapDisplay,curveArr,nCurvePts,nCurves,isCurveClosed,cvScalar(35,35,142),lineWidth);
				cvCircle(mapDisplay,cvPoint(nodes[i].p.getXCoord()-xMin,yMax-nodes[i].p.getYCoord()),1,CV_RGB (142,35,35),2);
			}
			else{
				cvPolyLine(mapDisplay,curveArr,nCurvePts,nCurves,isCurveClosed,cvScalar(255,0,0),lineWidth);
				cvCircle(mapDisplay,cvPoint(nodes[i].p.getXCoord()-xMin,yMax-nodes[i].p.getYCoord()),1,CV_RGB (0,0,255),2);
			}

		}
}
//~ void drawGraph(IplImage *mapDisplay,std::vector<Node> nodes,int xMin,int xMax,int yMin,int yMax){
	//~ //	Then draw the connections
		//~ for(unsigned int i=0;i<nodes.size();i++){
			//~ for(unsigned int j=0;j<nodes[i].neigh.size();j++){
				//~ vector<PixelCoords> tempv=getLine(nodes[i].p,nodes[i].neigh[j]);
				//~ for(unsigned int k=0;k<tempv.size();k++){
					//~ drawCross(mapDisplay,tempv[k].getXCoord()-xMin,yMax-tempv[k].getYCoord(),1,cvScalar(46,139,87),"");
				//~ }
			//~ }
		//~ }
	//	First draw the nodes		
//~ 
		//~ for(unsigned int i=0;i<nodes.size();i++){
//~ 
			//~ int x1=nodes[i].p.getXCoord()-xMin-5;
			//~ int y1=yMax-nodes[i].p.getYCoord()-5;
			//~ int x2=nodes[i].p.getXCoord()-xMin+5;
			//~ int y2=yMax-nodes[i].p.getYCoord()-5;
			//~ int x3=nodes[i].p.getXCoord()-xMin+5;
			//~ int y3=yMax-nodes[i].p.getYCoord()+5;
			//~ int x4=nodes[i].p.getXCoord()-xMin-5;
			//~ int y4=yMax-nodes[i].p.getYCoord()+5;
			//~ CvPoint  curve1[]={x1,y1, x2,y2, x3,y3, x4,y4};
			//~ CvPoint* curveArr[1]={curve1};
			//~ int      nCurvePts[1]={4};
			//~ int      nCurves=1;
			//~ int      isCurveClosed=1;
			//~ int      lineWidth=1;
			//~ 
			//~ if(nodes[i].neigh.size()==1){
				//~ cvPolyLine(mapDisplay,curveArr,nCurvePts,nCurves,isCurveClosed,cvScalar(35,35,142),lineWidth);
				//~ cvCircle(mapDisplay,cvPoint(nodes[i].p.getXCoord()-xMin,yMax-nodes[i].p.getYCoord()),1,CV_RGB (142,35,35),2);
			//~ }
			//~ else{
				//~ cvPolyLine(mapDisplay,curveArr,nCurvePts,nCurves,isCurveClosed,cvScalar(255,0,0),lineWidth);
				//~ cvCircle(mapDisplay,cvPoint(nodes[i].p.getXCoord()-xMin,yMax-nodes[i].p.getYCoord()),1,CV_RGB (0,0,255),2);
			//~ }
//~ 
		//~ }
//~ }

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
	
	mapAtt.robotPose.dx=0;
	mapAtt.robotPose.dy=0;
	
	mapAtt.map=map;

	Coverage coverage(mapAtt.getHeight(), mapAtt.getWidth());
	//-----------------------------------
	makeDummyCoveragePatch(map,coverage.coverage,2046,2046,rand()%2000);
	for(unsigned int i=0;i<55;i++){
		int x=0,y=0;
		while(map[x][y]<130){
			x=prevxMin+rand()%(prevxMax-prevxMin);
			y=prevyMin+rand()%(prevyMax-prevyMin);
		}
		makeDummyCoveragePatch(map,coverage.coverage,x,y,rand()%300);
	}
	//--------------------------------
	ExplorationTargetSelector targetSelector(mapAtt, coverage);
	PixelCoords target;
	
	
	targetSelector.selectTarget(&target);
	
	CvScalar pixel;
	
	image mapSmall(prevxMax-prevxMin,prevyMax-prevyMin,"mapsmall",3);
	for(unsigned int i=0;i<prevxMax-prevxMin;i++)
		for(unsigned int j=0;j<prevyMax-prevyMin;j++){
			pixel.val[0]=pixel.val[1]=pixel.val[2]=map[i+prevxMin][j+prevyMin];
			if(coverage.coverage[i+prevxMin][j+prevyMin]>0)
				pixel.val[0]=255-coverage.coverage[i+prevxMin][j+prevyMin];
			mapSmall.setPoint(i,j,pixel);
			//---draw voronoi
			//~ if(targetSelector.voronoiNodes.voronoi.voronoi[i+prevxMin][j+prevyMin]){
				//~ pixel.val[0]=pixel.val[1]=0;
				//~ pixel.val[2]=255;
				//~ mapSmall.setPoint(i,j,pixel);
			//~ }
		}
		
		
		
	//---draw voronoi topological graph
	drawGraph(mapSmall.display,targetSelector.voronoiNodes.nodes,prevxMin,prevxMax,prevyMin,prevyMax);
	
	
	int xx,yy;
	
	//---draw robotPose
	pixel.val[0]=0;
	pixel.val[1]=205;
	pixel.val[2]=0;
	xx=mapAtt.robotPose.dx + 2048 - prevxMin;
	yy=mapAtt.robotPose.dy + 2048 - prevyMin;
	
	mapSmall.drawCross( mapAtt.robotPose.dx + 2048 - prevxMin, mapAtt.robotPose.dy + 2048 - prevyMin,20, pixel);
	for(unsigned int i=0;i<3;i++){
		cvCircle(mapSmall.display,cvPoint( xx,mapSmall.display->height-yy-1),0.3/0.02*(1.5*i+1),CV_RGB (0,205,0),2);
	}
	//-----------------
	
	
	//---draw target
	pixel.val[0] =0;
	pixel.val[1]=0;
	pixel.val[2]=255;
	xx = target.getXCoord() - prevxMin;
	yy = target.getYCoord() - prevyMin;
	
	mapSmall.drawCross( xx, yy, 20, pixel);
	for(int i=0;i<3;i++){
		cvCircle(mapSmall.display,cvPoint(xx,mapSmall.display->height-yy-1),0.1/0.02*(1.5*i+1),CV_RGB (205,0,0),2);
	}
	//------------------
	
	
	//---draw closest unexplored
	pixel.val[0]=205;
	pixel.val[1]=0;
	pixel.val[2]=0;
	xx= targetSelector.closest.getXCoord() - prevxMin;
	yy= targetSelector.closest.getYCoord() - prevyMin;
	
	mapSmall.drawCross( targetSelector.closest.getXCoord() - prevxMin, targetSelector.closest.getYCoord() - prevyMin,20, pixel);
	for(int i=0;i<3;i++){
		cvCircle(mapSmall.display, cvPoint( xx,mapSmall.display->height-yy-1),0.1/0.02*(2*i+1),CV_RGB (0,0,205),2);
	}
	//------------------
	
	//---draw coverage limits
	pixel.val[0]=0;
	pixel.val[1]=205;
	pixel.val[2]=0;
	for (unsigned int i=0; i<targetSelector.closestUnexploredTargetSelector.coverageLimits.size(); i++){
		mapSmall.setPoint(targetSelector.closestUnexploredTargetSelector.coverageLimits[i].getXCoord() - prevxMin, targetSelector.closestUnexploredTargetSelector.coverageLimits[i].getYCoord() - prevyMin,pixel);
	}
	//------------------
	
	//--draw coverage path
	for (unsigned int i=0; i<targetSelector.covPath.size(); i++){
		mapSmall.drawCross( targetSelector.covPath[i].getXCoord() - prevxMin, targetSelector.covPath[i].getYCoord() - prevyMin,1, pixel);
		xx= targetSelector.covPath[i].getXCoord() - prevxMin;
		yy= targetSelector.covPath[i].getYCoord() - prevyMin;
		cvCircle(mapSmall.display, cvPoint( xx,mapSmall.display->height-yy-1),0.01/0.02*(2*1+1),CV_RGB (205,0,205),2);
	}
	//------------------
	
	//--draw path
	for (unsigned int i=0; i<targetSelector.goal.size(); i++){
		mapSmall.drawCross( targetSelector.goal[i].getXCoord() - prevxMin, targetSelector.goal[i].getYCoord() - prevyMin,1, pixel);
		xx= targetSelector.goal[i].getXCoord() - prevxMin;
		yy= targetSelector.goal[i].getYCoord() - prevyMin;
		cvCircle(mapSmall.display, cvPoint( xx,mapSmall.display->height-yy-1),0.01/0.02*(2*1+1),CV_RGB (205,0,0),2);
	}
	//------------------
	
	
	mapSmall.saveImage("smallXartis.png");
	mapSmall.show(0);
}
	
	
	
