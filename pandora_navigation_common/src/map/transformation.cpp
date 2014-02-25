#include "pandora_navigation_common/map/transformation.h"

Transformation::Transformation(void){
	//~ dx=0;
	//~ dy=0;
	//~ theta=0.0;
	clear();
}


/**
@brief
Overloading of +=

@param op2 The other Transformation

@return The pointer of the self object
**/
const Transformation & Transformation::operator+=(const Transformation &transformation){
	dx+=transformation.dx;
	dy+=transformation.dy;
	theta+=transformation.theta;
	return *this;
}

/**
@brief
Clears the Transformation

@return nothing
**/
void Transformation::clear(void){
	dx=0;
	dy=0;
	theta=0;
}

