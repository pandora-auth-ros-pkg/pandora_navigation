#ifndef IMAGE_H
#define IMAGE_H

#include <cv.h>
#include <highgui.h>
//~ #include <cxmisc.h>
//~ #include <cvaux.h>
#include <iostream>
//~ #include <opencv2/objdetect/objdetect.hpp>
//~ #include <opencv2/highgui/highgui.hpp>
//~ #include <opencv2/imgproc/imgproc.hpp>


class image{
	public:
		std::string title;
		//~ typedef cv::CLabPixelType IplImage *display;
		 IplImage *display;
		
		image(int width,int height,std::string title,int depth);
		image(int width,int height,std::string title,char **data);
		image(int width,int height,std::string title,unsigned char **data);
		image(int width,int height,std::string title,int **data);
		image(int width,int height,std::string title,bool **data);
		
		image(std::string file);
		
		//~ image(void){}
		
		void show(int usec);
		void destroy(void);
		void saveImage(std::string file);
		
		//~ void writeDataToPointer(bool **p);

		void writeDataToPointer(unsigned char **p);

		
		void setData(char **data);
		void setData(bool **data);
		void setData(unsigned char **data);
		void setData(int **data);
		void setPoint(int x,int y,CvScalar pixel);
		void drawCross(int x,int y,unsigned int size,CvScalar pixel);
		void printText(int x,int y,std::string text); 
};

#endif

