#include <iostream>
#include "test/Image.h"
#include "map/map_attributes.h"
#include "highgui.h"
#include "cv.h"
#include <iostream>
#include "navigation/path_planner/pandora_path_planner.h"
#define MAP_SIZE 4096
#include "test/test_utilities.h"
#include "geometry_msgs/PoseStamped.h"

#define d_PI_DOUBLE     		6.283185308


int main(){
	
	MapAttributes mapAtt(MAP_SIZE,MAP_SIZE);

	initMapFromImage("map1.png",&mapAtt);
	PixelCoords pose = PixelCoords(mapAtt.robotPose.dx, mapAtt.robotPose.dy);
	
	Voronoi voronoi(&mapAtt,true);
	
	int prevxMax = mapAtt.prevxMax ;
	int prevyMax = mapAtt.prevyMax ;
	int prevxMin = mapAtt.prevxMin ;
	int prevyMin = mapAtt.prevyMin ;
	
	//-----------------------------------
	
	//~ //target for RRT Tree
	PixelCoords target = PixelCoords( 2048+148, 2048+250 );
		
	//-----------------------------------.
	
	geometry_msgs::PoseStamped poseStampedGoal = transformPixelCoords2PoseStamped(target);
	
	std::vector<geometry_msgs::PoseStamped> plan;
	plan.clear();
	
	
	//~ PandoraPathPlanner pathPlanner(&mapAtt);
	
	TreePathGenerator treePathGenerator(&mapAtt,&voronoi) ; 
	
	treePathGenerator.generatePath( poseStampedGoal, plan) ;
	
	//~ pathPlanner.makePlan(pose, target, plan);
	
	
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
	
	//Draw Partition Graph
	//~ drawGraph(mapSmall.display,pathPlanner.partitionGraphPathGenerator.partitionNodes.nodes,prevxMin,prevxMax,prevyMin,prevyMax);
	
	//************************************* Draw robotNeighbors and targetNeighbors **************************
	//~ pixel.val[0] =0;
	//~ pixel.val[1]=0;
	//~ pixel.val[2]=255;
	//~ 
	//~ 
	//~ for ( unsigned int i=0; i< pathPlanner.partitionGraphPathGenerator.partitionNodes.robotNeighbors.size();i++){
		//~ 
		//~ int id = pathPlanner.partitionGraphPathGenerator.partitionNodes.robotNeighbors[i];
		//~ 
		//~ mapSmall.drawCross( pathPlanner.partitionGraphPathGenerator.partitionNodes.nodes[id].p.getXCoord() - prevxMin, pathPlanner.partitionGraphPathGenerator.partitionNodes.nodes[id].p.getYCoord() - prevyMin,5, pixel);
		//~ 
	//~ }
	//~ 
	//~ for ( unsigned int i=0; i< pathPlanner.partitionGraphPathGenerator.partitionNodes.targetNeighbors.size();i++){
		//~ 
		//~ int id = pathPlanner.partitionGraphPathGenerator.partitionNodes.targetNeighbors[i];
		//~ 
		//~ mapSmall.drawCross( pathPlanner.partitionGraphPathGenerator.partitionNodes.nodes[id].p.getXCoord() - prevxMin, pathPlanner.partitionGraphPathGenerator.partitionNodes.nodes[id].p.getYCoord() - prevyMin,5, pixel);
		//~ 
	//~ }
	//********************************************************************************************************
	
	
	//Draw the hole tree
	pixel.val[0] =0;
	pixel.val[1]=0;
	pixel.val[2]=0;
	for ( std::map<int, TreeNode*>::iterator it=treePathGenerator.robotTree.checkNodes.begin(); it!=treePathGenerator.robotTree.checkNodes.end(); ++it ){//i<pathPlanner.treePathGenerator.robotTree.checkNodes.size();  i++
		//int treeX =pathPlanner.treePathGenerator.robotTree.checkNodes[i].p.getXCoord();
		int treeX = it->second->p.getXCoord();
		//int treeY =pathPlanner.treePathGenerator.robotTree.checkNodes[i].p.getYCoord();
		int treeY = it->second->p.getYCoord();
		
		mapSmall.drawCross( treeX - prevxMin , treeY  - prevyMin, 1 ,pixel);
	}
	
	std::vector<PixelCoords> planInPixelCoords;
	
	cout << " size = " << plan.size() << endl;
	
	for(int ii=0;ii<plan.size();ii++){
		planInPixelCoords.push_back(transformPoseStamped2PixelCoords(plan[ii]));
		//~ cout << " yo :" <<plan[ii] << endl ;  
	}
	
	//draw path
	pixel.val[0] =0;
	pixel.val[1]=255;
	pixel.val[2]=0;
	for ( unsigned int i=0; i<plan.size(); i++)
		mapSmall.drawCross( planInPixelCoords[i].getXCoord() - prevxMin , planInPixelCoords[i].getYCoord()  - prevyMin, 1 ,pixel);
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
	
	
	
