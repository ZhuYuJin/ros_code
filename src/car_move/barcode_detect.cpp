
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/legacy/legacy.hpp>
#include <opencv2/opencv.hpp>

bool sort_by_point_num(const cv::vector<cv::Point>& v1, const cv::vector<cv::Point>& v2){
	return v1.size() >= v2.size();
}

int main(int argc, char **argv){

	ros::init(argc, argv, "barcode_detect");

	cv::namedWindow("test");

	cv::Mat img = cv::imread("/home/zhuyujin/Downloads/test.jpg");
    cv::Mat gray;
    cv::cvtColor(img, gray, CV_BGR2GRAY);

    cv::Mat gradX, gradY, gradient, blur, thresh;

    cv::Sobel(gray, gradX, CV_32F, 1, 0, -1);
    cv::Sobel(gray, gradY, CV_32F, 0, 1, -1);
    cv::subtract(gradX, gradY, gradient);
    cv::convertScaleAbs(gradient, gradient);

    cv::blur(gradient, blur, cv::Size(9,9));

    cv::threshold(blur, thresh, 225, 225, cv::THRESH_BINARY);

    cv::Mat kernel, closed;

    kernel = getStructuringElement(cv::MORPH_RECT, cv::Size(21, 7));
    morphologyEx(thresh, closed, cv::MORPH_CLOSE, kernel);

    cv::erode(closed, closed, cv::Mat(), cv::Point(-1, -1), 4);
    cv::dilate(closed, closed, cv::Mat(), cv::Point(-1, -1), 4);

    cv::vector< cv::vector<cv::Point> > contours;  
    cv::findContours(closed, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    sort(contours.begin(), contours.end(), sort_by_point_num);

    cv::RotatedRect rect = cv::minAreaRect(cv::Mat(contours[0]));
    cv::Rect brect = rect.boundingRect();

    cv::Point2f vertices[4];
	rect.points(vertices);
    for(int i = 0; i < sizeof(vertices); i++){
    	cv::circle(img, vertices[i], 5, cv::Scalar(0, 0, 255), 2);
    }

    cv::rectangle(img, brect, cv::Scalar(255,0,0), 2);

    // cv::drawContours(img, contours, -1, cv::Scalar(255), 3);

    cv::imshow("test", img);
    cvWaitKey(0);

    cv::destroyWindow("test");

	return 0;
}