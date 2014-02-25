#include "pandora_navigation_common/misc/pixelcoords.h"

PixelCoords::PixelCoords()
{
	//~ _imXCoord = D_DEFAULT;
   //~ _imYCoord = D_DEFAULT;
	_imXCoord = -1;
   _imYCoord = -1;
}

/**
@brief
Constructor

@param xCoord The x coordinate
@param yCoord The y coordinate

@return Nothing
**/
PixelCoords::PixelCoords(int xCoord, int yCoord)
{  _imXCoord = xCoord;
   _imYCoord = yCoord;
}

/**
@brief
Getter of Y coord

@return The y coord
**/
int PixelCoords::getYCoord()
{  return _imYCoord; }


/**
@brief
Getter of X coord

@return The X coord
**/
int PixelCoords::getXCoord()
{  return _imXCoord; }

/**
@brief
Setter of X coord

@param xCoord The x coordinate

@return void
**/
void PixelCoords::setXCoord(int xCoord)
{  _imXCoord = xCoord;
}

/**
@brief
Setter of Y coord

@param yCoord The y coordinate

@return void
**/
void PixelCoords::setYCoord(int yCoord)
{  _imYCoord = yCoord;
}

/**
@brief
Setter of both coords

@param xCoord The x coordinate
@param yCoord The y coordinate

@return void
**/
void PixelCoords::setCoords(int xCoord, int yCoord)
{  _imXCoord = xCoord;
   _imYCoord = yCoord;
}

/**
@brief
Setter of coords

@param newCoords The coords to be copied

@return void
**/
void PixelCoords::setCoords(PixelCoords newCoords)
{  _imXCoord = newCoords.getXCoord();
   _imYCoord = newCoords.getYCoord();
}

/**
@brief
Compares a cPixelCoord to another (basically like ==)

@param anotherCoord The coord to be compared with

@return True if equal
**/
bool PixelCoords::compareTo(PixelCoords anotherCoord)
{   if((_imXCoord == anotherCoord.getXCoord()) &&
       (_imYCoord == anotherCoord.getYCoord()))
        return true;
    return false;
}

/**
@brief
Calculates the distance from another coordinate

@param anotherCoord The other coord

@return The distance
**/
float PixelCoords::computeDistanceFrom(PixelCoords anotherCoord)
{   float xDist = (float)(_imXCoord - anotherCoord.getXCoord());
    float yDist = (float)(_imYCoord - anotherCoord.getYCoord());

    return sqrt((xDist*xDist) + (yDist*yDist));
}

/**
@brief
Calculates the squared distance from another coordinate

@param anotherCoord The other coord

@return The distance
**/
float PixelCoords::computeSqrDistFrom(PixelCoords anotherCoord)
{   float xDist = (float)(_imXCoord - anotherCoord.getXCoord());
    float yDist = (float)(_imYCoord - anotherCoord.getYCoord());

    return ((xDist*xDist) + (yDist*yDist));
}

/**
@brief
Overloading of =

@param obj The other coord

@return The pointer of the self object
**/
PixelCoords& PixelCoords::operator=(const PixelCoords &pixelCoords){
	this->_imXCoord=pixelCoords._imXCoord;
	this->_imYCoord=pixelCoords._imYCoord;
	return *this;
}

/**
@brief
Overloading of +

@param obj The other coord

@return The sum of the pixelCoords
**/
PixelCoords PixelCoords::operator+(const PixelCoords &pixelCoords){
	PixelCoords ret;
	ret._imXCoord=this->_imXCoord+pixelCoords._imXCoord;
	ret._imYCoord=this->_imYCoord+pixelCoords._imYCoord;
	return ret;
}

/**
@brief
Overloading of +=

@param obj The other coord

@return The pointer of the self object
**/
PixelCoords& PixelCoords::operator+=(const PixelCoords &pixelCoords){
	this->_imXCoord+=pixelCoords._imXCoord;
	this->_imYCoord+=pixelCoords._imYCoord;
	return *this;
}


PixelCoords PixelCoords::operator-(const PixelCoords &pixelCoords){
	PixelCoords ret;
	ret._imXCoord=this->_imXCoord-pixelCoords._imXCoord;
	ret._imYCoord=this->_imYCoord-pixelCoords._imYCoord;
	return ret;
}


/**
@brief
Overloading of *

@param obj The other coord
 
@return The propagation of one pixelCoord with a constant
**/
PixelCoords PixelCoords::operator*(float n){
	PixelCoords ret;
	ret._imXCoord=this->_imXCoord*n;
	ret._imYCoord=this->_imYCoord*n;
	return ret;
 
}

/**
@brief
Overloading of / by integer

@param div The divisor

@return The pointer of the self object
**/
PixelCoords& PixelCoords::operator/(const int div){
	this->_imXCoord/=div;
	this->_imYCoord/=div;
	return *this;
}

/**
@brief
Overloading of ==

@param obj The other coord

@return True if this and obj are equal
**/
bool PixelCoords::operator==(const PixelCoords &pixelCoords){
	if(this->_imXCoord==pixelCoords._imXCoord && this->_imYCoord==pixelCoords._imYCoord) return true;
	return false;
}

/**
@brief
Overloading of !=

@param obj The other coord

@return True if this and obj are equal
**/
bool PixelCoords::operator!=(const PixelCoords &pixelCoords){
	if(this->_imXCoord==pixelCoords._imXCoord && this->_imYCoord==pixelCoords._imYCoord) return false;
	return true;
}
