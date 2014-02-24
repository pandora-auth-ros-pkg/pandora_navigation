#include <iostream>
#include "test/Image.h"
#include "map/map_attributes.h"
#include "highgui.h"
#include "cv.h"
#include <iostream>
#include "navigation/path_planner/path_generator/voronoi_path_generator/voronoi_path_generator.h"
#define MAP_SIZE 4096

#include "geometry_msgs/PoseStamped.h"
#include "test/test_utilities.h"

#define d_PI_DOUBLE     		6.283185308


int main(){
	
	MapAttributes mapAtt(MAP_SIZE,MAP_SIZE);

	initMapFromImage("map1.png",&mapAtt);
	
	PixelCoords pose = PixelCoords(mapAtt.robotPose.dx, mapAtt.robotPose.dy);
	
	Voronoi voronoi(&mapAtt,true);
	
	//-----------------------------------
	
	//~ //target for Voronoi
	PixelCoords target = PixelCoords( 2048-250, 2048+250 );
	
	//-----------------------------------.
	
	geometry_msgs::PoseStamped poseStampedGoal = transformPixelCoords2PoseStamped(target);
	
	std::vector<geometry_msgs::PoseStamped> plan;
	plan.clear();
	
	VoronoiPathGenerator voronoiPathGenerator(&mapAtt,&voronoi);
	
	voronoiPathGenerator.generatePath( poseStampedGoal, plan) ;

		
	int prevxMax = mapAtt.prevxMax ;
	int prevyMax = mapAtt.prevyMax ;
	int prevxMin = mapAtt.prevxMin ;
	int prevyMin = mapAtt.prevyMin ;
	
	CvScalar pixel;
	
	image mapSmall(prevxMax-prevxMin,prevyMax-prevyMin,"mapsmall",3);
	for(unsigned int i=0;i<prevxMax-prevxMin;i++){
		for(unsigned int j=0;j<prevyMax-prevyMin;j++){
			pixel.val[0]=pixel.val[1]=pixel.val[2]=mapAtt.map[i+prevxMin][j+prevyMin];
			mapSmall.setPoint(i,j,pixel);
			//Draw voronoi
			//~ if(pathPlanner.voronoiPathGenerator.voronoi.voronoi[i+prevxMin][j+prevyMin]){
				//~ pixel.val[0]=pixel.val[1]=0;
				//~ pixel.val[2]=255;
				//~ mapSmall.setPoint(i,j,pixel);
			//~ }
		}
	}
	

	
	//draw path
	pixel.val[0] =0;
	pixel.val[1]=255;
	pixel.val[2]=0;
	for ( unsigned int i=0; i<plan.size(); i++)
		mapSmall.drawCross( (int)(plan[i].pose.position.x) - prevxMin , (int)(plan[i].pose.position.y)  - prevyMin, 1 ,pixel);
	//draw robotPose
	pixel.val[0] = pixel.val[1]=255;
	pixel.val[2]=0;
	mapSmall.drawCross( mapAtt.robotPose.dx + 2048 - prevxMin, mapAtt.robotPose.dy + 2048 - prevyMin,5, pixel);
	//draw target
	pixel.val[0] =255;
	pixel.val[1]=0;
	pixel.val[2]=255;
	mapSmall.drawCross( target.getXCoord() - prevxMin, target.getYCoord() - prevyMin,5, pixel);
	//save and show map
	mapSmall.saveImage("smallXartis.png");
	mapSmall.show(0);
}
	
	
	
