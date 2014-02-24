#ifndef NAVIGATOR_POINT_H
#define NAVIGATOR_POINT_H


	//---M---//
	#ifndef MAP_SIZE_H
	#define MAP_SIZE_H
		#define MAP_HEIGHT				4096		//!< Map height
		#define MAP_WIDTH				4096		//!< Map width
		#define MAP_SIZE				4096 		//!< Maximum map size
		#define	START_X					MAP_HEIGHT/2		//!< Start robot X coordinate in map
		#define	START_Y 				MAP_HEIGHT/2		//!< Start robot Y coordinate in map
	#endif
//~ #define MAP_SIZE 4096
/*! \struct Point
    \brief Holds information about a Point
*/
class Point{
	public:
	int x;         //!< The x coordinate
	int y;         //!< The y coordinate
    
	float theta;   //!< Angle (used in laser rays)
	
	/**
		@brief Void constructor
		@param void
		@return void
	 **/
	Point(){}
	
	 /**
		@brief Constructor
		@param x [int] : X coordinate of point
		@param y [int] : Y coordinate of point
		@return void
	 **/
	Point(int x,int y){
		this->x=x; 
		this->y=y;
	} 
	
	/**
	@brief	Overloading of == operator
	@param other The other object
	@return True if this==other
	**/
	bool operator==(const Point &point) const{
		if(x==point.x && y==point.y) return true;
		return false;
	}
	
	/**
	@brief	Overloading of < operator
	@param other The other object
	@return True if this<other
	**/
	bool operator<(const Point &point) const{
		if((y*MAP_SIZE+x)<(point.y*MAP_SIZE+point.x)) return true;
		return false;
	}
};


#endif
