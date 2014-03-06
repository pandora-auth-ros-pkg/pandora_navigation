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


#ifndef PIXELCOORDS_H
#define PIXELCOORDS_H

#include "cmath"

/*! \class PixelCoords
    \brief Basic method of a coordinate pair 
*/
class PixelCoords
{  private:
     int _imXCoord;     //!< The x coord
     int _imYCoord;     //!< The y coord
   public:
     /* constructors */
    /**
		@brief Void constructor
		@param void
		@return void
	 **/
     PixelCoords();
     
     /**
    @brief Constructor
	@param xCoord [int] : X coordinate of pixel
	@param yCoord [int] : Y coordinate of pixel
	@return void
	 **/
     PixelCoords(int xCoord, int yCoord);  
     
     /* Getters */
    
    /**
	@brief Returns X Coord
	@param void
	@return void
	**/
     int getXCoord(); 
             
      /**
	@brief Returns Y Coord
	@param void
	@return void
	**/      
     int getYCoord();        
            
     /* Setters */
    /**
	@brief Sets X coordinate
	@param xCoord [int] : The x coordinate
	@return void
	**/ 
     void setXCoord(int xCoord);
     
     /**
     @brief Sets Y coordinate
	@param xCoord [int] : The y coordinate
	@return void
	**/ 
     void setYCoord(int yCoord);              
     
	  /**
	@brief
	Setter of both coords

	@param xCoord The x coordinate
	@param yCoord The y coordinate

	@return void
	**/
     void setCoords(int xCoord, int yCoord);   
     
     /**
	@brief
	Setter of coords

	@param newCoords The coords to be copied

	@return void
	**/
     void setCoords(PixelCoords newCoords);    
     
     /* Misc */
    /**
	@brief
	Compares a cPixelCoord to another (basically like ==)

	@param anotherCoord The coord to be compared with

	@return True if equal
	**/
     bool compareTo(PixelCoords anotherCoord);         
     
     /**
	@brief
	Calculates the distance from another coordinate

	@param anotherCoord The other coord

	@return The distance
	**/
     float computeDistanceFrom(PixelCoords anotherCoord);     
     
     /**
	@brief
	Calculates the squared distance from another coordinate

	@param anotherCoord The other coord

	@return The distance
	**/
     float computeSqrDistFrom(PixelCoords anotherCoord);      //!< Calculates the square of the distance between two coords
     
     /**
	@brief
	Overloading of =

	@param obj The other coord

	@return The pointer of the self object
	**/
     PixelCoords& operator=(const PixelCoords &pixelCoords);  //!< Overloading of = operator
     
     /**
	@brief
	Overloading of +

	@param obj The other coord

	@return The sum of the pixelCoords
	**/
     PixelCoords operator+(const PixelCoords &pixelCoords);   
    
    /**
	@brief
	Overloading of +=

	@param obj The other coord

	@return The pointer of the self object
	**/
     PixelCoords& operator+=(const PixelCoords &pixelCoords); 
     
     /**
	@brief
	Overloading of -

	@param obj The other coord

	@return The difference of the pixelCoords
	**/
     PixelCoords operator-(const PixelCoords &pixelCoords);   
     
     /**
	@brief
	Overloading of *

	@param obj The other coord
	 
	@return The propagation of one pixelCoord with a constant
	**/
     PixelCoords operator*(float n);					 
     
     /**
	@brief
	Overloading of / by integer

	@param div The divisor

	@return The pointer of the self object
	**/
     PixelCoords& operator/(const int div);   			
     
     /**
	@brief
	Overloading of ==

	@param obj The other coord

	@return True if this and obj are equal
	**/
     bool operator==(const PixelCoords &pixelCoords);      
     
     /**
	@brief
	Overloading of !=

	@param obj The other coord

	@return True if this and obj are not equal
	**/
     bool operator!=(const PixelCoords &pixelCoords);     
     
     
	template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
			ar & _imXCoord;
			ar & _imYCoord;
	}
     
};


#endif

