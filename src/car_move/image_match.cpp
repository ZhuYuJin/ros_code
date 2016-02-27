
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/legacy/legacy.hpp>
#include <opencv2/opencv.hpp>
#include <vector>  
#include <iostream>
#include "RobustMatcher.h"  

using namespace std; 

static const std::string OPENCV_WINDOW = "Monitor";
static const std::string RESULT_WINDOW = "Result";
 
class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &ImageConverter::imageDivMatch, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
    cv::namedWindow(RESULT_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
    cv::destroyWindow(RESULT_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
    
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }

  void imageDivMatch(const sensor_msgs::ImageConstPtr& msg)
  {
    int numKeyPoints = 1500;

    //Instantiate robust matcher

    RobustMatcher rmatcher;

    //instantiate detector, extractor, matcher

    cv::Ptr<cv::FeatureDetector> detector = new cv::OrbFeatureDetector(numKeyPoints);
    cv::Ptr<cv::DescriptorExtractor> extractor = new cv::OrbDescriptorExtractor;
    cv::Ptr<cv::DescriptorMatcher> matcher = new cv::BruteForceMatcher<cv::HammingLUT>;

    rmatcher.setFeatureDetector(detector);
    rmatcher.setDescriptorExtractor(extractor);
    rmatcher.setDescriptorMatcher(matcher);

    //Load input image detect keypoints

    cv::Mat img1;
    std::vector<cv::KeyPoint> img1_keypoints;
    cv::Mat img1_descriptors;
    cv::Mat img2;
    std::vector<cv::KeyPoint> img2_keypoints;
    cv::Mat img2_descriptors;

    std::vector<cv::DMatch>  matches;

    img1 = cv::imread("/home/zhuyujin/Downloads/main.jpg", CV_LOAD_IMAGE_GRAYSCALE);
    /*img2 = cv::imread("C:\\temp\\PyramidPatternTest.bmp", CV_LOAD_IMAGE_GRAYSCALE);*/
    //img2 = cv::imread("C:\\temp\\test1.jpg", CV_LOAD_IMAGE_GRAYSCALE);
    img2 = cv::imread("/home/zhuyujin/Downloads/main.jpg", CV_LOAD_IMAGE_GRAYSCALE);

    rmatcher.match(img1, img2, matches, img1_keypoints, img2_keypoints);
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}