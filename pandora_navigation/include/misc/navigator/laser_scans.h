#ifndef LASER_SCAN_H
#define LASER_SCAN_H


#ifndef LASER_RAYS_H
#define LASER_RAYS_H
	#define LASER_RAYS     			726			//!< Number of laser rays
#endif

#include "misc/navigator/point.h"
/*! \struct LaserScan
    \brief Holds a laser scan
*/
class LaserScan{
	public:
	float scan[LASER_RAYS];         //!< Martix that holds the laser scan (distances)
	float density[LASER_RAYS-1];    //!< Hepl matrix for the measurements density (used in SLAM)
	
	Point p[LASER_RAYS];            //!< Matrix that holds the scan (points)
};


#endif

