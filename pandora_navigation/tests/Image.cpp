#include "test/Image.h"

image::image(int width,int height,std::string title,int depth){
	display = cvCreateImage(cvSize(width,height), IPL_DEPTH_8U,depth);
	this->title=title;
}

image::image(std::string file){
	display=cvLoadImage(file.c_str(),0);
}

//~ void image::writeDataToPointer(bool **p){
	//~ int height     = display->height;
	//~ int width      = display->width;
//~ 
	//~ int step       = display->widthStep/sizeof(uchar);
	//~ uchar* data    = (uchar *)display->imageData;
	//~ for(int i=0;i<width;i++)
		//~ for(int j=0;j<height;j++)
			//~ p[i][j]=data[i*step+j];	
//~ }


//~ image::image(int width,int height,string title,char **data){
	//~ display = cvCreateImage(cvSize(width,height), IPL_DEPTH_8U,1);
	//~ this->title=title;
	//~ CvScalar pixel;
	//~ for(int i=0;i<width;i++)
		//~ for(int j=0;j<height;j++){
			//~ pixel.val[0]=data[i][j];
			//~ cvSet2D(display,display->height-1-j,i, pixel);
		//~ }
//~ }
//~ 
//~ 
//~ 
//~ image::image(int width,int height,string title,unsigned char **data){
	//~ display = cvCreateImage(cvSize(width,height), IPL_DEPTH_8U,1);
	//~ this->title=title;
	//~ CvScalar pixel;
	//~ for(int i=0;i<width;i++)
		//~ for(int j=0;j<height;j++){
			//~ pixel.val[0]=data[i][j];
			//~ cvSet2D(display,display->height-1-j,i, pixel);
		//~ }
//~ }
//~ 
//~ image::image(int width,int height,string title,bool **data){
	//~ display = cvCreateImage(cvSize(width,height), IPL_DEPTH_8U,1);
	//~ this->title=title;
	//~ CvScalar pixel;
	//~ for(int i=0;i<width;i++)
		//~ for(int j=0;j<height;j++){
			//~ pixel.val[0]=data[i][j]*255;
			//~ cvSet2D(display,display->height-1-j,i, pixel);
		//~ }
//~ }

void image::setData(char **data){
	CvScalar pixel;
	for(int i=0;i<display->width;i++)
		for(int j=0;j<display->height;j++){
			pixel.val[0]=data[i][j];
			cvSet2D(display,display->height-1-j,i, pixel);
		}
}

void image::setData(unsigned char **data){
	CvScalar pixel;
	for(int i=0;i<display->width;i++)
		for(int j=0;j<display->height;j++){
			pixel.val[0]=data[i][j];
			cvSet2D(display,display->height-1-j,i, pixel);
		}
}

void image::setData(bool **data){
	CvScalar pixel;
	for(int i=0;i<display->width;i++)
		for(int j=0;j<display->height;j++){
			if (data[i][j]){
				pixel.val[0]=255;
				cvSet2D(display,display->height-1-j,i, pixel);
				
			}
		}
}

//~ image::image(int width,int height,string title,int **data){
	//~ display = cvCreateImage(cvSize(width,height), IPL_DEPTH_8U,1);
	//~ this->title=title;
	//~ int max=0;
	//~ 
	//~ for(int i=0;i<width;i++)
		//~ for(int j=0;j<height;j++)
			//~ if(data[i][j]>max) 
				//~ max=data[i][j];
		//~ 
	//~ CvScalar pixel;
	//~ for(int i=0;i<width;i++)
		//~ for(int j=0;j<height;j++){
			//~ pixel.val[0]=(int)((float)data[i][j]*255.0/max);
			//~ cvSet2D(display,display->height-1-j,i, pixel);
		//~ }
//~ }

void image::saveImage(std::string file){
	cvSaveImage(file.c_str(),display);
}

void image::setData(int **data){
	int max=0;
	
	for(int i=0;i<display->width;i++)
		for(int j=0;j<display->height;j++)
			if(data[i][j]>max) 
				max=data[i][j];
		
	CvScalar pixel;
	for(int i=0;i<display->width;i++)
		for(int j=0;j<display->height;j++){
			pixel.val[0]=(int)((float)data[i][j]*255.0/max);
			cvSet2D(display,display->height-1-j,i, pixel);
		}
}

void image::show(int usec){
	cvNamedWindow(title.c_str());//,CV_WINDOW_AUTOSIZE);
	cvShowImage( title.c_str(), display );
    cvWaitKey(usec);
}

void image::destroy(void){
	cvReleaseImage(&display);
}

void image::setPoint(int x,int y,CvScalar pixel){
	cvSet2D(display,display->height-1-y,x, pixel);
}

void image::drawCross(int x,int y,unsigned int size,CvScalar pixel){
	cvSet2D(display,display->height-1-y,x,pixel);
	for(unsigned int i=1;i<=size;i++){
		cvSet2D(display,display->height-1-(y+i),x,pixel);
		cvSet2D(display,display->height-1-(y-i),x,pixel);
		cvSet2D(display,display->height-1-y,x+i,pixel);
		cvSet2D(display,display->height-1-y,x-i,pixel);
	}
}

void image::printText(int x,int y,std::string text){
	CvFont font;
	cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX, 0.6,0.6,0,2);
	cvPutText(display,text.c_str(),cvPoint(x+10,display->height-1-y),&font,cvScalar(255,255,255));
}


void image::writeDataToPointer(unsigned char **p){
	int height     = display->height;
	int width      = display->width;

	int step       = display->widthStep/sizeof(uchar);
	uchar* data    = (uchar *)display->imageData;
	for(int i=0;i<width;i++)
		for(int j=0;j<height;j++)
			p[i][j]=data[i*step+j];	
}
