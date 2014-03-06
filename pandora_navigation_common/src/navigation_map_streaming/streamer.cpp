/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, P.A.N.D.O.R.A. Team.
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
* Author: Manos Tsardoulias
* Author: Aris Thallas
*********************************************************************/

#include <cv.h>
#include <highgui.h>
#include <iostream>
#include <cstdlib>
#include <cmath>
#include <vector>
#include <fstream>
#include <string>
#include <sstream>
#include <map>
#include <pthread.h>
#include <set>
#include <sys/time.h>

#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include <sensor_msgs/Image.h>

#include <opencv/cvwimage.h>
#include <cv_bridge/cv_bridge.h>
//-----------Services------------//
#include "pandora_navigation_communications/navigationMapSrv.h"


using namespace std;

/*! \struct Point
    \brief Holds information about a Point
*/
struct Point{
	int x,y;
	Point(int x,int y){
		this->x=x;
		this->y=y;
	}
};

/**
@brief Finds the straight line between two points
@param x1 The first point - x coord
@param x2 The second point - x coord
@param y1 The first point - y coord
@param y2 The second point - y coord
@return A vector with the line points
 **/
std::vector<Point> getLine(int x1,int x2,int y1,int y2){
	vector<Point> ret;
	ret.clear();
	int temp;
	if(x1==x2){
		if(y1>y2){
			temp=y1;
			y1=y2;
			y2=temp;
		}
		for(int i=y1;i<y2;i++)
			ret.push_back(Point(x1,i));
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
			ret.push_back(Point(i,(int(l*i-l*x1+(float)y1))));
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
			ret.push_back(Point((i-(float)y1+l*x1)/l,i));
			
			
		return ret;
	}
}

/**
@brief Streams the navigation map to GUI
@return True on success
 **/
int main (int argc,char **argv){

	
	
	ros::init(argc, argv, "navigationMapSrv");
	IplImage *mapDisplay;
	std::vector<unsigned char> map,coverage,voronoi;
	std::vector<int> voronodesx,voronodesy,goalsx,goalsy,coverageLimitsx,coverageLimitsy;
	int width,height,xx,yy;
	int xRobot,yRobot;
	float angleRobot;
	std::string mapTitle;
	ros::ServiceClient navigSrvClient,targetSrvClient;
	pandora_navigation_communications::navigationMapSrv navigMapSrv,targetSelMapSrv;
	ros::NodeHandle n;
	navigSrvClient=n.serviceClient<pandora_navigation_communications::navigationMapSrv>("NavigationMapSrv/navigationController");
	targetSrvClient=n.serviceClient<pandora_navigation_communications::navigationMapSrv>("NavigationMapSrv/targetSelectorController");
	mapTitle="Map from Navigation";
	ROS_DEBUG("Map displayer initialised");
	mapDisplay = cvCreateImage(cvSize(100,100), IPL_DEPTH_8U,3);

		
	IplImage *hazmats[5];
	hazmats[0]=cvLoadImage("patterns/1.png");
	if(!hazmats[0]){
		ROS_WARN("[NAVIGATION_MAP] Could not load pattern 1");
	}
	hazmats[1]=cvLoadImage("patterns/2.png");
	if(!hazmats[1]){
		ROS_WARN("[NAVIGATION_MAP] Could not load pattern 2");
	}
	hazmats[2]=cvLoadImage("patterns/3.png");
	if(!hazmats[2]){
		ROS_WARN("[NAVIGATION_MAP] Could not load pattern 3");
	}
	hazmats[3]=cvLoadImage("patterns/4.png");
	if(!hazmats[3]){
		ROS_WARN("[NAVIGATION_MAP] Could not load pattern 4");
	}
	hazmats[4]=cvLoadImage("patterns/5.png");
	if(!hazmats[4]){
		ROS_WARN("[NAVIGATION_MAP] Could not load pattern 5");
	}
	hazmats[5]=cvLoadImage("patterns/6.png");
	if(!hazmats[5]){
		ROS_WARN("[NAVIGATION_MAP] Could not load pattern 6");
	}
	
	image_transport::Publisher _cameraPublisher = image_transport::ImageTransport(n).advertise("/navigation/navigationStreaming", 1);
	//~ ros::Publisher _cameraPublisher = ros::NodeHandle().advertise<sensor_msgs::Image>("navigationStreaming", 1);
	
	sleep(5);
	while(ros::ok()){
		
		ROS_DEBUG("Going to ask map");
		
		usleep(1000000);
		while(!navigSrvClient.call(navigMapSrv))
			usleep(10000);
		while(!targetSrvClient.call(targetSelMapSrv))
			usleep(10000);
		
		ROS_DEBUG("Got map");
			
		//~ ROS_ERROR("Got navigation map");
		
		
		width=navigMapSrv.response.xsize;
		height=navigMapSrv.response.ysize;
		xRobot=navigMapSrv.response.xRobot;
		yRobot=navigMapSrv.response.yRobot;
		angleRobot=navigMapSrv.response.angleRobot;
		
		ROS_DEBUG("width = %d \n",width);
		ROS_DEBUG("height = %d \n",height);
		ROS_DEBUG("xRobot = %d \n",xRobot);
		ROS_DEBUG("yRobot = %d \n",yRobot);
		ROS_DEBUG("angleRobot = %f \n",angleRobot);
		
		
		map=navigMapSrv.response.map;
		coverage=targetSelMapSrv.response.coverage;
		voronoi=navigMapSrv.response.voronoi;
		
		cvReleaseImage(&mapDisplay);
		mapDisplay=cvCreateImage(cvSize(width,height), IPL_DEPTH_8U,3);

		voronodesx=targetSelMapSrv.response.voronodesx;
		voronodesy=targetSelMapSrv.response.voronodesy;
		
		goalsx=navigMapSrv.response.goalsx;
		goalsy=navigMapSrv.response.goalsy;
		
		coverageLimitsx=targetSelMapSrv.response.coverageLimitsx;
		coverageLimitsy=targetSelMapSrv.response.coverageLimitsy;
		
		CvScalar pixel;

		//	Draw map and coverage
		for(int i=0;i<width;i++){
			for(int j=0;j<height;j++){
				pixel.val[1]=pixel.val[2]=pixel.val[0]=map[i*height+j];
				if(coverage[i*height+j]!=0) pixel.val[0]=255-coverage[i*height+j]; 
				cvSet2D(mapDisplay,height-1-j,i, pixel);
			}
		}
		
		pixel.val[0]=pixel.val[1]=0;
		pixel.val[2]=255;
		for(int i=0;i<width;i++)
			for(int j=0;j<height;j++)
				if(voronoi[i*height+j])
					cvSet2D(mapDisplay,height-1-j,i, pixel);
					
		//	Decomp
		for(unsigned int i=0;i<navigMapSrv.response.decompNeighborsFirst.size();i++){
			if(navigMapSrv.response.decompx[navigMapSrv.response.decompNeighborsFirst[i]]<0 || navigMapSrv.response.decompx[navigMapSrv.response.decompNeighborsFirst[i]]>=width){
				ROS_WARN("[NAVIGATION_MAP %d] Wrong vals",__LINE__);
				continue;
			}
			if(navigMapSrv.response.decompx[navigMapSrv.response.decompNeighborsLast[i]]<0 || navigMapSrv.response.decompx[navigMapSrv.response.decompNeighborsLast[i]]>=width){
				ROS_WARN("[NAVIGATION_MAP %d] Wrong vals",__LINE__);
				continue;
			}
			if(navigMapSrv.response.decompy[navigMapSrv.response.decompNeighborsFirst[i]]<0 || navigMapSrv.response.decompy[navigMapSrv.response.decompNeighborsFirst[i]]>=height){
				ROS_WARN("[NAVIGATION_MAP %d] Wrong vals",__LINE__);
				continue;
			}
			if(navigMapSrv.response.decompy[navigMapSrv.response.decompNeighborsLast[i]]<0 || navigMapSrv.response.decompy[navigMapSrv.response.decompNeighborsLast[i]]>=height){
				ROS_WARN("[NAVIGATION_MAP %d] Wrong vals",__LINE__);
				continue;
			}
			vector<Point> tempv=getLine(navigMapSrv.response.decompx[navigMapSrv.response.decompNeighborsFirst[i]],navigMapSrv.response.decompx[navigMapSrv.response.decompNeighborsLast[i]],navigMapSrv.response.decompy[navigMapSrv.response.decompNeighborsFirst[i]],navigMapSrv.response.decompy[navigMapSrv.response.decompNeighborsLast[i]]);
			for(unsigned int k=0;k<tempv.size();k++){
				cvSet2D(mapDisplay,height-1-tempv[k].y,tempv[k].x,cvScalar(200,255,200));
			}
		}

		//	Coverage Limits
		pixel.val[0]= 0;
		pixel.val[1]= 150;
		pixel.val[2]= 0;
		int x,y;
		for(unsigned int i=0;i<coverageLimitsx.size();i++){
			x=coverageLimitsx[i];
			y=coverageLimitsy[i];

			cvSet2D(mapDisplay,height-1-y,x,pixel);	
		}	

		//	Robot Pose
		if(xRobot>=0 && xRobot<width && yRobot>=0 && yRobot<height){
			pixel.val[0]= 0;
			pixel.val[1]= 0;
			pixel.val[2]=255;
			cvSet2D(mapDisplay,height-1-yRobot,xRobot,pixel);
			for( int i=1;i<=10;i++){
				cvSet2D(mapDisplay,height-1-(yRobot+i),xRobot,pixel);
				cvSet2D(mapDisplay,height-1-(yRobot-i),xRobot,pixel);
				cvSet2D(mapDisplay,height-1-(yRobot),xRobot+i,pixel);
				cvSet2D(mapDisplay,height-1-(yRobot),xRobot-i,pixel);
			}
			for(int i=0;i<2;i++){
				cvCircle(mapDisplay,cvPoint(xRobot,height-1-yRobot),1.0/0.02*(i+1),CV_RGB (0,100,0),0.1);
			}
			
			int xr,yr;
			xr=xRobot+50*cos(angleRobot);
			yr=yRobot+50*sin(angleRobot);
			//Draw robot angle
			vector<Point> tempv=getLine(xRobot,xr,yRobot,yr);
			for(unsigned int k=0;k<tempv.size();k++){
				cvSet2D(mapDisplay,height-1-(tempv[k].y),tempv[k].x,cvScalar(0,0,255));
				cvSet2D(mapDisplay,height-1-(tempv[k].y)-1,tempv[k].x,cvScalar(0,0,255));
				cvSet2D(mapDisplay,height-1-(tempv[k].y),tempv[k].x-1,cvScalar(0,0,255));
			}
		}
		else{
			ROS_WARN("[NAVIGATION_MAP %d] Wrong vals",__LINE__);
		}
		
		//	Robot Trajectory
		pixel.val[0]= 0;
		pixel.val[1]= 0;
		pixel.val[2]=255;
		for ( int i=0; i<navigMapSrv.response.trajectorySize; i++ ){
			if(navigMapSrv.response.xRobotTrajectory[i]>=0 && navigMapSrv.response.xRobotTrajectory[i]<width && navigMapSrv.response.yRobotTrajectory[i]>=0 && navigMapSrv.response.yRobotTrajectory[i]<height){
				cvSet2D(	mapDisplay,
							height-1-(navigMapSrv.response.yRobotTrajectory[i]),
							navigMapSrv.response.xRobotTrajectory[i],
							pixel);
			}
			else{
				ROS_WARN("[NAVIGATION_MAP %d] Wrong trajectory vals",__LINE__);
			}
		}
		
		//	Voronodes
		int voronodesSize=targetSelMapSrv.response.voronodesx.size();
		for(int i=0;i<voronodesSize;i++){
			if(voronodesx[i]<4 && voronodesx[i]>(width-4) && voronodesy[i]<4 && voronodesy[i]>(height-4)){
				ROS_WARN("[NAVIGATION_MAP %d] Wrong vals",__LINE__);
				continue;
			}
			for( int j=1;j<=4;j++){
				cvSet2D(mapDisplay,height-1-(voronodesy[i]+j),voronodesx[i]+j,cvScalar(0,50,0));
				cvSet2D(mapDisplay,height-1-(voronodesy[i]-j),voronodesx[i]-j,cvScalar(0,50,0));
				cvSet2D(mapDisplay,height-1-(voronodesy[i]+j),voronodesx[i]-j,cvScalar(0,50,0));
				cvSet2D(mapDisplay,height-1-(voronodesy[i]-j),voronodesx[i]+j,cvScalar(0,50,0));
			}
			//~ std::string s;
			//~ std::stringstream out;
			//~ out << i;
			//~ s = out.str();
			//~ CvFont font;
			//~ cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX, 0.6,0.6,0,1.5);
			//~ cvPutText(mapDisplay,s.c_str(),cvPoint(voronodesx[i]+10,height-1-voronodesy[i]),&font,cvScalar(255,0,0));
		}
		for(unsigned int i=0;i<targetSelMapSrv.response.neighborsFirst.size();i++){
			vector<Point> tempv=getLine(voronodesx[targetSelMapSrv.response.neighborsFirst[i]],voronodesx[targetSelMapSrv.response.neighborsLast[i]],voronodesy[targetSelMapSrv.response.neighborsFirst[i]],voronodesy[targetSelMapSrv.response.neighborsLast[i]]);
			for(unsigned int k=0;k<tempv.size();k++){
				cvSet2D(mapDisplay,height-1-(tempv[k].y),tempv[k].x,cvScalar(0,100,0));
			}
		}
		
	
		
		//	Goals
		if(goalsx.size()!=0){
			pixel.val[0]= 255;
			pixel.val[1]= 0;
			pixel.val[2]= 0;
			for(unsigned int i=0;i<goalsx.size();i++){
				xx=goalsx[i];
				yy=goalsy[i];
				cvCircle(mapDisplay,cvPoint(xx,height-1-yy),7,CV_RGB (200,100,0),0.1);
				cvCircle(mapDisplay,cvPoint(xx,height-1-yy),4,CV_RGB (200,100,0),0.1);
			}
			xx=goalsx[goalsx.size()-1];
			yy=goalsy[goalsy.size()-1];
			for(int i=0;i<3;i++){
				cvCircle(mapDisplay,cvPoint(xx,height-1-yy),0.5/0.02*(i+1),CV_RGB (200,100,0),0.1);
			}
			for(unsigned int i=0;i<goalsx.size()-1;i++){
				vector<Point> tempv=getLine(goalsx[i],goalsx[i+1],goalsy[i],goalsy[i+1]);
				for(unsigned int k=0;k<tempv.size();k++){
					cvSet2D(mapDisplay,height-1-tempv[k].y,tempv[k].x,cvScalar(255,100,0));
					cvSet2D(mapDisplay,height-1-(tempv[k].y-1),tempv[k].x,cvScalar(255,100,0));
					cvSet2D(mapDisplay,height-1-(tempv[k].y+1),tempv[k].x,cvScalar(255,100,0));
					cvSet2D(mapDisplay,height-1-tempv[k].y,tempv[k].x-1,cvScalar(255,100,0));
					cvSet2D(mapDisplay,height-1-tempv[k].y,tempv[k].x+1,cvScalar(255,100,0));
				}
			}
		}
		
		//	victims
		//If there are found victims
		if(navigMapSrv.response.victimsx.size()!=0){
			for(unsigned int i=0;i<navigMapSrv.response.victimsx.size();i++){
				if(navigMapSrv.response.victimsx[i]<=0 || navigMapSrv.response.victimsx[i]>=width || navigMapSrv.response.victimsy[i]<=0 || navigMapSrv.response.victimsy[i]>=height){
					ROS_WARN("[NAVIGATION_MAP %d] Wrong vals",__LINE__);
					continue;
				}
				if(navigMapSrv.response.valid[i]==0){
					cvCircle(mapDisplay,cvPoint(navigMapSrv.response.victimsx[i],height-1-navigMapSrv.response.victimsy[i]),7,CV_RGB (255,0,0),1);
					cvCircle(mapDisplay,cvPoint(navigMapSrv.response.victimsx[i],height-1-navigMapSrv.response.victimsy[i]),5,CV_RGB (255,0,0),1);
				}
				else{
					cvCircle(mapDisplay,cvPoint(navigMapSrv.response.victimsx[i],height-1-navigMapSrv.response.victimsy[i]),7,CV_RGB (0,255,0),1);
					cvCircle(mapDisplay,cvPoint(navigMapSrv.response.victimsx[i],height-1-navigMapSrv.response.victimsy[i]),5,CV_RGB (0,255,0),1);
				}
					
			}
		}
		
		if(navigMapSrv.response.hazmatx.size()!=0){
			for(unsigned int i=0;i<navigMapSrv.response.hazmatx.size();i++){
				if(navigMapSrv.response.hazmatx[i]<=0 || navigMapSrv.response.hazmatx[i]>=width || navigMapSrv.response.hazmaty[i]<=0 || navigMapSrv.response.hazmaty[i]>=height){
					ROS_WARN("[NAVIGATION_MAP %d] Wrong vals",__LINE__);
					continue;
				}
				
				std::string s;
				CvFont font;
				if((int)navigMapSrv.response.pattern[i]==5 || ((int)navigMapSrv.response.pattern[i]>=13 && (int)navigMapSrv.response.pattern[i]<=18))
					s="E";
				else{
					s=std::string("Hz:");
					std::stringstream out5;
					out5 << (int)navigMapSrv.response.pattern[i];;
					s+=out5.str();
				}
				
				cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX, 0.4,0.4,0,1.5);	
				cvPutText(mapDisplay,s.c_str(),cvPoint(navigMapSrv.response.hazmatx[i]+5,height-1-(5+navigMapSrv.response.hazmaty[i])),&font,cvScalar(0,150,0));	
				
				cvCircle(mapDisplay,cvPoint(navigMapSrv.response.hazmatx[i],height-1-navigMapSrv.response.hazmaty[i]),3,CV_RGB (0,150,0),1);	
				cvCircle(mapDisplay,cvPoint(navigMapSrv.response.hazmatx[i],height-1-navigMapSrv.response.hazmaty[i]),2,CV_RGB (0,150,0),1);
				cvCircle(mapDisplay,cvPoint(navigMapSrv.response.hazmatx[i],height-1-navigMapSrv.response.hazmaty[i]),5,CV_RGB (0,150,0),1);
				cvCircle(mapDisplay,cvPoint(navigMapSrv.response.hazmatx[i],height-1-navigMapSrv.response.hazmaty[i]),4,CV_RGB (0,150,0),1);	
		
			}
		}
		
		ROS_DEBUG("[NAVIGATION_MAP %d] Im here",__LINE__);
		//Print labels of sensor
		for(unsigned int i=0;i<navigMapSrv.response.victimsx.size();i++){
			std::string s;
			CvFont font;
			s=navigMapSrv.response.sensorIDFound[i];
			cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX, 0.5,0.5,0,1);	
			cvPutText(mapDisplay,s.c_str(),cvPoint(navigMapSrv.response.victimsx[i]+10,height-1-(navigMapSrv.response.victimsy[i])+8),&font,cvScalar(0,0,255));
			
		}
		ROS_DEBUG("[NAVIGATION_MAP %d] Im here",__LINE__);	
		//If there are victims to go
		for(unsigned int i=0;i<navigMapSrv.response.victimsToGox.size();i++){
			if(navigMapSrv.response.victimsToGox[i]<0 || navigMapSrv.response.victimsToGox[i]>=width || navigMapSrv.response.victimsToGoy[i]<0 || navigMapSrv.response.victimsToGoy[i]>=height){
				ROS_WARN("[NAVIGATION_MAP %d] Wrong vals",__LINE__);
				continue;
			}
			
			cvCircle(mapDisplay,cvPoint(navigMapSrv.response.victimsToGox[i],height-1-navigMapSrv.response.victimsToGoy[i]),7,CV_RGB (0,0,255),1);
			cvCircle(mapDisplay,cvPoint(navigMapSrv.response.victimsToGox[i],height-1-navigMapSrv.response.victimsToGoy[i]),5,CV_RGB (0,0,255),1);
		}
		
		//Print labels of sensor
		for(unsigned int i=0;i<navigMapSrv.response.victimsToGox.size();i++){
			std::string s;
			s=navigMapSrv.response.sensorIDGo[i];
			CvFont font;
			cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX, 0.3,0.3,0,1.3);	
			cvPutText(mapDisplay,s.c_str(),cvPoint(navigMapSrv.response.victimsToGox[i]+10,height-1-(navigMapSrv.response.victimsToGoy[i])+8),&font,cvScalar(0,0,255));
			
		}
			
		//Approach point of current victim
		if(navigMapSrv.response.victimsToGox.size()>0){
			
			if(navigMapSrv.response.victimAppPointx<0 || navigMapSrv.response.victimAppPointx>=width || navigMapSrv.response.victimAppPointy<0 || navigMapSrv.response.victimAppPointy>=height){
				ROS_WARN("[NAVIGATION_MAP %d] Wrong vals",__LINE__);
			}
			else
				cvCircle(mapDisplay,cvPoint(navigMapSrv.response.victimAppPointx,height-1-navigMapSrv.response.victimAppPointy),4,CV_RGB (150,20,150),1);
			
			for(unsigned int i=0;i<navigMapSrv.response.victimAppPointsx.size();i++){
				
				if(navigMapSrv.response.victimAppPointsx[i]<0 || navigMapSrv.response.victimAppPointsx[i]>=width || navigMapSrv.response.victimAppPointsy[i]<0 || navigMapSrv.response.victimAppPointsy[i]>=height){
					ROS_WARN("[NAVIGATION_MAP %d] Wrong vals",__LINE__);
					continue;
				}
				
				cvCircle(mapDisplay,cvPoint(navigMapSrv.response.victimAppPointsx[i],height-1-navigMapSrv.response.victimAppPointsy[i]),4,CV_RGB (150,20,150),1);
			}
		}
		std::string s;
		CvFont font;
		
		static long internalCounter=0;
		internalCounter++;
		
		std::stringstream out2;
		out2 << internalCounter;
		s = out2.str();
		cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX, 0.6,0.6,0,1);
		cvPutText(mapDisplay,s.c_str(),cvPoint(10,30),&font,cvScalar(0,0,200));

		// Stream
//		cv::WImageBuffer3_b image( (IplImage*)cvClone(mapDisplay) );
//		sensor_msgs::ImagePtr msg = sensor_msgs::CvBridge::cvToImgMsg(image.Ipl(), "bgr8");
//		msg->header.stamp = ros::Time::now();
//		_cameraPublisher.publish(msg);

		cv::Mat imgMat(mapDisplay,false);		//Change from IplImage* to cv::Mat, no data copy
		cv_bridge::CvImage msg;
		msg.header.stamp   = ros::Time::now();
		msg.encoding = sensor_msgs::image_encodings::BGR8;
		msg.image    = imgMat.clone();
		_cameraPublisher.publish(msg.toImageMsg());

	}
	return 0;
}

