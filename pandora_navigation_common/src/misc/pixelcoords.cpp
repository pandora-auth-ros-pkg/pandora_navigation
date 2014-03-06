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
