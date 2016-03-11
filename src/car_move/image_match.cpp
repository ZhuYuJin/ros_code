
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
static const std::string TEST_WINDOW = "Test";
 
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
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &ImageConverter::imageFeatureMatch2, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
    cv::namedWindow(RESULT_WINDOW);
    cv::namedWindow(TEST_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
    cv::destroyWindow(RESULT_WINDOW);
    cv::destroyWindow(TEST_WINDOW);
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

    // img1 = cv_ptr->image;
    // cvtColor(img1, img1, CV_BGR2GRAY);  
    img1 = cv::imread("/home/zhuyujin/Downloads/main2.jpg", CV_LOAD_IMAGE_GRAYSCALE);
    img2 = cv::imread("/home/zhuyujin/Downloads/main4.jpg", CV_LOAD_IMAGE_GRAYSCALE);

    rmatcher.match(img1, img2, matches, img1_keypoints, img2_keypoints);
    cv::waitKey(3);
  }

  void imageFeatureMatch(const sensor_msgs::ImageConstPtr& msg){
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

    cv::Mat origin_img, cam_img, cam_img_8u;
    origin_img = cv::imread("/home/zhuyujin/Downloads/main.jpg", CV_8U);
    cvtColor(cv_ptr->image, cam_img, CV_BGR2GRAY);  

// cv::imshow(TEST_WINDOW, cam_img);
// cv::waitKey(3);

    // double minVal, maxVal;
    // minMaxLoc(cam_img_32f, &minVal, &maxVal);
    // cam_img_32f.convertTo(cam_img_8u, CV_8U, 255.0/(maxVal - minVal), -minVal);

    cv::Mat origin_img_feature, cam_img_feature;
    origin_img_feature = featureDetector(origin_img);
    cam_img_feature = featureDetector(cam_img);

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

    // img1 = cv_ptr->image;
    // cvtColor(img1, img1, CV_BGR2GRAY);  
    img1 = origin_img_feature;
    img2 = cam_img_feature;

    rmatcher.match(img1, img2, matches, img1_keypoints, img2_keypoints);
    cv::waitKey(3);

    // // Update GUI Window
    // cv::imshow(OPENCV_WINDOW, origin_img);
    // cv::imshow(RESULT_WINDOW, cam_img);
    // cv::waitKey(3);
    
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }

  void imageFeatureMatch2(const sensor_msgs::ImageConstPtr& msg){
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

    cv::Mat origin_img, cam_img, cam_img_8u;
    origin_img = cv::imread("/home/zhuyujin/Downloads/main.jpg", CV_8U);
    cvtColor(cv_ptr->image, cam_img, CV_BGR2GRAY);  

    cv::Mat origin_img_feature, cam_img_feature;
    origin_img_feature = featureDetector(origin_img);
    cam_img_feature = featureDetector(cam_img);

    // // Update GUI Window
    // cv::imshow(OPENCV_WINDOW, origin_img_feature);
    // cv::imshow(RESULT_WINDOW, cam_img_feature);
    // cv::waitKey(3);

    //Method2
    cv::Mat image1, image2;
    image1 = origin_img_feature;
    image2 = cam_img_feature;
    // image1 = cv::imread("/home/zhuyujin/Downloads/main.jpg", CV_LOAD_IMAGE_GRAYSCALE);
    // image2 = cv::imread("/home/zhuyujin/Downloads/main2.jpg", CV_LOAD_IMAGE_GRAYSCALE);

    // 检测surf特征点
    vector<cv::KeyPoint> keypoints1,keypoints2;     
    cv::SurfFeatureDetector detector(400);
    detector.detect(image1, keypoints1);
    detector.detect(image2, keypoints2);
    
    // 描述surf特征点
    cv::SurfDescriptorExtractor surfDesc;
    cv::Mat descriptros1,descriptros2;
    surfDesc.compute(image1,keypoints1,descriptros1);
    surfDesc.compute(image2,keypoints2,descriptros2);
    
    // 计算匹配点数
    cv::BruteForceMatcher< cv::L2<float> >matcher;
    vector<cv::DMatch> matches;
    matcher.match(descriptros1,descriptros2,matches);
    // std::nth_element(matches.begin(),matches.begin()+24,matches.end());
    // matches.erase(matches.begin()+25,matches.end());
    
    // 画出匹配图
    cv::Mat imageMatches;
    cv::drawMatches(image1,keypoints1,image2,keypoints2,matches,
        imageMatches,cv::Scalar(255,0,0));

    cv::imshow(RESULT_WINDOW, imageMatches);
    cv::waitKey(3);
  }

  cv::Mat featureDetector(cv::Mat img_8u){
    cv::Mat copy = img_8u;

    cv::Mat cross(5, 5, CV_8U);
    cv::Mat diamond(5, 5, CV_8U, cv::Scalar(1));
    cv::Mat square(5, 5, CV_8U, cv::Scalar(1));
    cv::Mat x(5, 5, CV_8U, cv::Scalar(0));

    for (int i = 0; i < 5; i++)
    {
      cross.at<uchar>(2, i) = 1;
      cross.at<uchar>(i, 2) = 1;
    }

    diamond.at<uchar>(0, 0) = 0;
    diamond.at<uchar>(0, 1) = 0;
    diamond.at<uchar>(1, 0) = 0;
    diamond.at<uchar>(4, 4) = 0;
    diamond.at<uchar>(3, 4) = 0;
    diamond.at<uchar>(4, 3) = 0;
    diamond.at<uchar>(4, 0) = 0;
    diamond.at<uchar>(4, 1) = 0;
    diamond.at<uchar>(3, 0) = 0;
    diamond.at<uchar>(0, 4) = 0;
    diamond.at<uchar>(4, 1) = 0;
    diamond.at<uchar>(3, 0) = 0;
    diamond.at<uchar>(0, 4) = 0;
    diamond.at<uchar>(0, 3) = 0;
    diamond.at<uchar>(1, 4) = 0;

    for (int i = 0; i < 5; i++)
    {
      x.at<uchar>(i, i) = 1;
      x.at<uchar>(4 - i, i) = 1;
    }

    cv::Mat result, result2;
    cv::dilate(copy, result, cross);
    cv::erode(result, result, diamond);
    cv::dilate(copy, result2, x);
    cv::erode(result2, result2, square);

    cv::absdiff(result2, result, result);

    //阈值化
    cv::threshold(result, result, 80, 255, cv::THRESH_BINARY);

    return result;
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}