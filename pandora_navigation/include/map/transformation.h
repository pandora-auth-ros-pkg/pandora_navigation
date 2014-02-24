#ifndef NAVIGATION_TRANSFORMATION_H
#define NAVIGATION_TRANSFORMATION_H

class Transformation{
	
	
public:
	//these need to be private
	int dx;            //!< The Dx
	int dy;            //!< The Dy
	int dz;
	float theta;       //!< The DThete
	float pitch;
	float roll;
	
	Transformation(void);
	
	/**
	@brief
	Overloading of +=

	@param op2 The other Transformation

	@return The pointer of the self object
	**/
	const Transformation & operator+=(const Transformation &transformation);  

	/**
	@brief
	Clears the Transformation

	@return nothing
	**/
	void clear(void);  //!< Makes the Transformation 0
};

#endif
