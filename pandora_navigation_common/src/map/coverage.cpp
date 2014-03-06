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

 /** 
  * File Description coverage (Source file) - Implements the coverage functionality
  * Contents:
		* Coverage::addSENSOR
		* Coverage::fixRobotPatch
		* Coverage::fixHeadPatch
		* Coverage::patchMapAt
  * Author: Navigation team
  */

#include "pandora_navigation_common/map/coverage.h"

using namespace std;

Coverage::Coverage(int Height, int Width){
	
	
	coverage=new unsigned char *[Height];
	for(unsigned int i=0;i<Height;i++)
		coverage[i]=new unsigned char [Width];
	for(unsigned int i=0;i<Height;i++){
		for(unsigned int j=0;j<Width;j++){
			coverage[i][j]=0;
		}
	}
	addSENSOR(ID_THERMAL_TPA,0,(int)(0.5/OCGD),41.0/180.0*D_PI,ID_ROBOT); 				//TPA 
	addSENSOR(ID_THERMAL_TPA,45.0/180.0*D_PI,(int)(0.5/OCGD),41.0/180.0*D_PI,ID_ROBOT); 	//TPA 
	addSENSOR(ID_THERMAL_TPA,-45.0/180.0*D_PI,(int)(0.5/OCGD),41.0/180.0*D_PI,ID_ROBOT);	//TPA 
	addSENSOR(ID_THERMAL_TPA,90.0/180.0*D_PI,(int)(0.5/OCGD),41.0/180.0*D_PI,ID_ROBOT); 	//TPA 
	addSENSOR(ID_THERMAL_TPA,-90.0/180.0*D_PI,(int)(0.5/OCGD),41.0/180.0*D_PI,ID_ROBOT);	//TPA 
	addSENSOR(ID_CAMERA,0,(int)(2.0/OCGD),D_PI,ID_HEAD);							//CAMERA 
	addSENSOR(ID_CO2,0,(int)(0.3/OCGD),D_PI_DOUBLE,ID_HEAD); 					//CO2
	addSENSOR(ID_SOUND,0,(int)(1.0/OCGD),D_PI_DOUBLE,ID_HEAD); 					//SOUND
	addSENSOR(ID_THERMAL_MLX,0,(int)(1.5/OCGD),10.0/180.0*D_PI,ID_HEAD); 					//MLX
	
	fixHeadPatch();
	fixRobotPatch();
}

Coverage::Coverage(void){
	
	addSENSOR(ID_THERMAL_TPA,0,(int)(0.5/OCGD),41.0/180.0*D_PI,ID_ROBOT); 				//TPA 
	addSENSOR(ID_THERMAL_TPA,45.0/180.0*D_PI,(int)(0.5/OCGD),41.0/180.0*D_PI,ID_ROBOT); 	//TPA 
	addSENSOR(ID_THERMAL_TPA,-45.0/180.0*D_PI,(int)(0.5/OCGD),41.0/180.0*D_PI,ID_ROBOT);	//TPA 
	addSENSOR(ID_THERMAL_TPA,90.0/180.0*D_PI,(int)(0.5/OCGD),41.0/180.0*D_PI,ID_ROBOT); 	//TPA 
	addSENSOR(ID_THERMAL_TPA,-90.0/180.0*D_PI,(int)(0.5/OCGD),41.0/180.0*D_PI,ID_ROBOT);	//TPA 
	addSENSOR(ID_CAMERA,0,(int)(2.0/OCGD),D_PI,ID_HEAD);							//CAMERA 
	addSENSOR(ID_CO2,0,(int)(0.3/OCGD),D_PI_DOUBLE,ID_HEAD); 					//CO2
	addSENSOR(ID_SOUND,0,(int)(1.0/OCGD),D_PI_DOUBLE,ID_HEAD); 					//SOUND
	addSENSOR(ID_THERMAL_MLX,0,(int)(1.5/OCGD),10.0/180.0*D_PI,ID_HEAD); 					//MLX
	
	coverage=new unsigned char *[MAP_SIZE];
	for(unsigned int i=0;i<MAP_SIZE;i++)
		coverage[i]=new unsigned char [MAP_SIZE];
	for(unsigned int i=0;i<MAP_SIZE;i++){
		for(unsigned int j=0;j<MAP_SIZE;j++){
			coverage[i][j]=0;
		}
	}
	fixHeadPatch();
	fixRobotPatch();
}
/**
@brief
Adds a Sensor in the coverage patch

@param sensorID The ID of the Sensor (declared in defines.h)
@param angle The angle of the Sensor according to the robot X axis
@param DOV Distance that the Sensor can work
@param AOV Angle that the Sensor measures
@param position The position of the Sensor, head or robots main body

@return void
**/
void Coverage::addSENSOR(char sensorID,float angle,float DOV,float AOV,char position){
	Sensor t;
	t.sensorID=sensorID;
	t.angle=angle;
	t.DOV=DOV;
	t.AOV=AOV;	
	sensors.push_back(t);
}

/**
@brief
Creates the patch. Must be called after the addSensor calls.

@return void
**/
void Coverage::fixRobotPatch(void){

	robotMaxX=0;
	robotMaxY=0;
	for(unsigned int i=0;i<ROBOT_SENSORS;i++)
		if(sensors[i].DOV>robotMaxX) 
			robotMaxX=sensors[i].DOV;
	robotMaxX=robotMaxY=2*robotMaxX+20;
	xstart=ystart=robotMaxX/2;
	robotPatch=new unsigned char*[robotMaxX];
	for(int i=0;i<robotMaxX;i++)
		robotPatch[i]=new unsigned char[robotMaxX];
	
	for(int i=0;i<robotMaxX;i++)
		for(int j=0;j<robotMaxX;j++)
			robotPatch[i][j]=0;
	
	for(unsigned int i=0;i<ROBOT_SENSORS;i++){
		float angleStart=sensors[i].angle-sensors[i].AOV/2;
		float angleStop=sensors[i].angle+sensors[i].AOV/2;
		float a,d,localGain;
		localGain=THERMAL_TPA_GAIN*GLOBAL_GAIN;
		a=angleStart;
		int x,y;
		while(a<angleStop){
			d=0;
			while(d<sensors[i].DOV){
				x=xstart+d*cos(a);
				y=ystart+d*sin(a);
				robotPatch[x][y]+=(255-robotPatch[x][y])/2*localGain;
				d++;
			}
			a+=0.02;
		}
	}

}


void Coverage::fixHeadPatch(void){

	headMaxX=0;
	headMaxY=0;
	for(unsigned int i=ROBOT_SENSORS;i<sensors.size();i++)
		if(sensors[i].DOV>headMaxX) 
			headMaxX=sensors[i].DOV;
	headMaxX=headMaxY=2*headMaxX+20;
	xstart=ystart=headMaxX/2;
	headPatch=new unsigned char*[headMaxX];
	for(int i=0;i<headMaxX;i++)
		headPatch[i]=new unsigned char[headMaxX];
	
	for(int i=0;i<headMaxX;i++)
		for(int j=0;j<headMaxX;j++)
			headPatch[i][j]=0;
	
	for(unsigned int i=ROBOT_SENSORS;i<sensors.size();i++){
		float angleStart=sensors[i].angle-sensors[i].AOV/2;
		float angleStop=sensors[i].angle+sensors[i].AOV/2;
		float a,d,localGain=0;
		
		switch(sensors[i].sensorID){
			case ID_THERMAL_MLX:
				localGain=THERMAL_MLX_GAIN*GLOBAL_GAIN;
				break;
			case ID_CAMERA:
				localGain=CAMERA_GAIN*GLOBAL_GAIN;
				break;
			case ID_CO2:
				localGain=CO2_GAIN*GLOBAL_GAIN;
				break;
			case ID_SOUND:
				localGain=SOUND_GAIN*GLOBAL_GAIN;
		}
		
		a=angleStart;
		int x,y;
		while(a<angleStop){
			d=0;
			while(d<sensors[i].DOV){
				x=xstart+d*cos(a);
				y=ystart+d*sin(a);
				headPatch[x][y]+=(255-headPatch[x][y])/2*localGain;
				d++;
			}
			a+=0.02;
		}
	}

}


void Coverage::flush(void){
	
	for(unsigned int i=0;i<MAP_SIZE;i++){
		for(unsigned int j=0;j<MAP_SIZE;j++){
			coverage[i][j]=0;
		}
	}
	
}

/**
@brief
Patches the patch to the map

@param x The robot's x coordinate - The center of the patch
@param y The robot's y coordinate - The center of the patch
@param theta The robots orientation
@param coverage The coverage map
@param map The map

@return void
**/
void Coverage::patchMapAt(int x,int y,float theta,unsigned char **map,int dX,int dY,float dTheta){
	//~ ROS_INFO_NAMED("coverage_patch","[Coverage %d] Updating map's coverage",__LINE__);
	
	float costh=cos(theta);
	float sinth=sin(theta);
	for(int i=-robotMaxX/2;i<robotMaxX/2;i++){
		for(int j=-robotMaxY/2;j<robotMaxY/2;j++){

			int tempx,tempy;
			tempx=i*costh-j*sinth+x;
			tempy=i*sinth+j*costh+y;
			
			if(tempx>=(int)MAP_SIZE || tempx<0 || tempy>=(int)MAP_SIZE || tempy<0) continue;
			if(map[tempx][tempy]<EMPTY_THRESHOLD) continue;
			
			coverage[tempx][tempy]+=(255-coverage[tempx][tempy])/2.0*(robotPatch[i+robotMaxX/2][j+robotMaxY/2]/255.0)*COVERAGE_GAIN;	
		}
	}
	
	
	float cosdTh=cos(theta+dTheta);
	float sindTh=sin(theta+dTheta);
	for(int i=-headMaxX/2;i<headMaxX/2;i++){
		for(int j=-headMaxY/2;j<headMaxY/2;j++){

			int tempdX,tempdY;
			tempdX=i*cosdTh-j*sindTh+x+dX;
			tempdY=i*sindTh+j*cosdTh+y+dY;
			
			if(tempdX>=(int)MAP_SIZE || tempdX<0 || tempdY>=(int)MAP_SIZE || tempdY<0) continue;
			if(map[tempdX][tempdY]<EMPTY_THRESHOLD) continue;
			
			coverage[tempdX][tempdY]+=(255-coverage[tempdX][tempdY])/2.0*(headPatch[i+headMaxX/2][j+headMaxY/2]/255.0)*COVERAGE_GAIN;	
		}
	}
	
}
