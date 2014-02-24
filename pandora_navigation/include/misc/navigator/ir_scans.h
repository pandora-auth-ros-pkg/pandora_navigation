#ifndef IR_SCAN_H
#define IR_SCAN_H


#include "misc/navigator/point.h"
/*! \struct IrScan
    \brief Holds an irs scan
*/

class IrScan{
	public:
	float distance[5];	//!< Martix that holds the ir scan (distances)
	float front;		//!< Front IR measurement
	float right;		//!< Right IR measurement
	float left;			//!< Left IR measurement
	float back;			//!< Back IR measurement
	float arm;			//!< Arm IR measurement
	
	Point p[5];			//!< Martix that holds the scan (Point)
};


#endif

