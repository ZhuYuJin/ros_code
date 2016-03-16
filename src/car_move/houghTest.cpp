#include "ros/ros.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/legacy/legacy.hpp>
#include <opencv2/opencv.hpp>

int main(int argc, char **argv){

	ros::init(argc, argv, "infraredTest");

	// IplImage* src = NULL;  
 //    IplImage* dst = NULL;  
 //    IplImage* color = NULL;  
  
 //    src = cvLoadImage ("/home/zhuyujin/Downloads/123.jpg", 1);  
 //    dst = cvCreateImage (cvGetSize(src), IPL_DEPTH_8U, 1);  
 //    if (src->nChannels == 1)  
 //    {  
 //        dst = cvCloneImage (src);  
 //    }  
 //    else  
 //    {  
 //        cvCvtColor (src, dst, CV_RGB2GRAY);  
 //    }  
  
 //    CvMemStorage* storage = cvCreateMemStorage (0);  
 //    cvSmooth (dst, dst, CV_GAUSSIAN, 5, 5);  
      
 //    CvSeq* circles = cvHoughCircles (dst, storage, CV_HOUGH_GRADIENT, 2, dst->width / 3, 300, 100, 0, 200);  
 //    color = cvCreateImage (cvGetSize(src), IPL_DEPTH_8U, 3);  
 //    cvCvtColor (dst, color, CV_GRAY2RGB);  
 //    for (int i = 0; i < circles->total; i++)  
 //    {  
 //        float* p = (float*)cvGetSeqElem (circles, i);  
 //        CvPoint pt = cvPoint (cvRound(p[0]), cvRound(p[1]));  
 //        cvCircle (color, pt, cvRound(p[2]), CV_RGB(255, 0, 0), 3, 8, 0);  
 //    }  
  
 //    cvNamedWindow ("src", 1);  
 //    cvShowImage ("src", src);  
 //    cvNamedWindow ("circle", 1);  
 //    cvShowImage ("circle", color);  
  
 //    cvWaitKey (0);  

 //    cvReleaseMemStorage (&storage);  
 //    cvReleaseImage (&src);  
 //    cvReleaseImage (&dst);  
 //    cvReleaseImage (&color); 

	CvMemStorage* storage = cvCreateMemStorage(0);  
	CvSeq* contours;  
	CvBox2D s;  
	char string1[22];  
	char string2[22];  
	char string3[22];  
	char string4[222];  
	IplImage* img = cvLoadImage( "/home/zhuyujin/Downloads/234.jpg" );  
	IplImage* gray = cvCreateImage(cvGetSize(img), 8, 1 );  
	IplImage* gray1 = cvCreateImage(cvGetSize(img), 8, 1 );   
	cvCvtColor(img, gray, CV_BGR2GRAY );  
	cvThreshold( gray, gray1 ,100, 255, CV_THRESH_BINARY );  
	cvNamedWindow( "Ellipse", 1 );  
	cvShowImage("Ellipse",gray1);  
	  
	int i=cvFindContours( gray1, storage, &contours, sizeof(CvContour),  
	                CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));  
	// itoa(i, string3, 10 );  
	for(CvSeq* c=contours; c!=NULL; c=c->h_next)  
	{  
		CvContour* r = *(CvContour**)cvGetSeqElem( contours, i );
		s=cvFitEllipse2(r);  
		s.angle=90-s.angle;  
		cvEllipseBox(img,s, CV_RGB(255,0,0),2, 8 , 0 );//画椭圆框  
	// itoa(s.size.width, string1, 10 );  
	// itoa(s.size.height, string2, 10 );  
	// strcat(string1,"  ");  
	// strcat(string1,string2);  
	// strcat(string4,string1);  
	// strcat(string4," | ");  
	}  
	  
	cvNamedWindow( "Ellipse22", 1 );  
	cvShowImage("Ellipse22",img);//原图上显示椭圆框  
	  
	// ShowMessage( AnsiString(" 共找到椭圆 ")  + string3 + AnsiString(" 个/n ")+  
	// AnsiString("椭圆的长短轴分别为:")  + string4 + AnsiString("像素 "));  

    cvReleaseMemStorage (&storage);  
    cvReleaseImage (&img);  
    cvReleaseImage (&gray);  
    cvReleaseImage (&gray1); 

    cvWaitKey (0);  

	return 0;
}