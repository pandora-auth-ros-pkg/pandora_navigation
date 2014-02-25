#ifndef SONAR_SCAN_H
#define SONAR_SCAN_H

#include "point.h"
/*! \struct SonarScan
    \brief Holds a sonar scan
*/
class SonarScan{
	public:
	float distance[6];	//!< Martix that holds the sonar scan (distances)
	float front;		//!< Front sonar measurement
	float right;		//!< Right sonar measurement
	float left;			//!< Left sonar measurement
	float backRight;	//!< Back right sonar measurement
	float backLeft;		//!< Back left sonar measurement
	float arm;			//!< Arm sonar measurement
	
	Point p[6];			//!< Martix that holds the scan (Point)
};


#endif
