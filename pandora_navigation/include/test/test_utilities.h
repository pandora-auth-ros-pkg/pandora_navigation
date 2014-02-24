#include <iostream>
#include "map/map_attributes.h"
#include "highgui.h"
#include "cv.h"
#include <iostream>
#include <vector>
#include "geometry_msgs/PoseStamped.h"

#include "nodes/node.h"
#include "test/Image.h"
#define MAP_SIZE 4096
#define d_PI_DOUBLE     		6.283185308

using namespace std;

std::vector<PixelCoords> getLine(PixelCoords p1,PixelCoords p2);

void makeDummyCoveragePatch(unsigned char **map,unsigned char **coverage,int x, int y,int d);

void drawCross(IplImage *mapDisplay,int x,int y, int size,CvScalar pixel,std::string func);

void drawGraph(IplImage *mapDisplay,std::vector<Node> nodes,int xMin,int xMax,int yMin,int yMax);

PixelCoords transformPoseStamped2PixelCoords(geometry_msgs::PoseStamped poseStamped);

geometry_msgs::PoseStamped transformPixelCoords2PoseStamped(PixelCoords pixelCoords);

void initMapFromImage(const string filename,MapAttributes* mapAtt);
