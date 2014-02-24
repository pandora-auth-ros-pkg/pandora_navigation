#ifndef VORONOI_H
#define VORONOI_H


#include "voronoi_defines.h"
#include "misc/pixelcoords.h"
#include "map/map_attributes.h"
#include "vector"

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

//TODO: create new constructor with bool argument for partitionGraph
//TODO: new function for brushCell calculation

class Voronoi{
	
private:
	
	//~ **bool voronoi;
	//~ int mapHeight,mapWidth;
	
	//computes voronoi and brushCell
	//void computeVoronoi(void);
	//void computeVoronoi(unsigned char **map, bool **isOnVoronoi, MapInfo info,unsigned int xMax, unsigned int xMin,unsigned int yMax,unsigned int yMin,float **brushCell,PixelCoords **parent);
	void computeVoronoi(unsigned char **map,int mapHeight, int mapWidth,unsigned int xMax, unsigned int xMin,unsigned int yMax,unsigned int yMin,float **brushCell);
	
	
	//Trims the voronoi edges by "contamination" concept
	void trimVoronoiEdges(int mapHeight, int mapWidth);
	//~ void trimVoronoiEdges(bool **voronoi, MapInfo &info);
	
	//Checks if a Point is a voronoi's edge
	//bool isEdge(int i, int j);
	bool isEdge(int i, int j, int mpaHeight, int mapWigth);
	//~ bool isEdge(bool **voronoi, int i, int j, MapInfo &info);
	
	
	bool performThinningIteration(unsigned char **img,unsigned char **img2,int width,int height);
	bool performPruningIteration(unsigned char **img,unsigned char **img2,int width,int height);
	void copyElements(unsigned char **img,unsigned char **img2,int width,int height);
	bool performFinalPruningIteration(unsigned char **img,unsigned char **img2,int width,int height);
	
	
public:
	
	template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
		
		for(int ii=0;ii<mapAttributes->getHeight();ii++){
			for(int jj=0;jj<mapAttributes->getWidth();jj++){
				
				ar & voronoi[ii][jj];
			
		}
	}
		
		
		
		
    }
	
	
	bool** voronoi;
	PixelCoords **parent;
	float** brushCell;
	
	MapAttributes* mapAttributes;
	
	//voronoi constructor. TODO: maybe add void constructor
	//Voronoi(void){}
	//~ void Voronoi(const MapAttributes* mapAttr);
	//Voronoi(const MapAttributes* mapAttr, bool brush=false);
	Voronoi(MapAttributes* mapAttr, bool brush=false);
	
	
	void fixVoronoi(void);
	void computeBrushCell(void);
	//~ void computeBrushCell(unsigned char **map,int mapHeight, int mapWidth,unsigned int xMax, unsigned int xMin,unsigned int yMax,unsigned int yMin);
	
	
};


#endif

	//~ TODO:remove these and edit functions
	//~ void setMapHeight(int height) { mapHeight = height };
	//~ void setMapWidth(int width) { mapWidth = width };
	//~ int getMapHeight(void) { return height };
	//~ int getMapWidth(void) { return width };
