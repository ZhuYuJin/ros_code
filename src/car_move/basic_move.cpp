// #include "RobotController.h"
#include <wiringPi.h>
#include "ros/ros.h"
#include "std_msgs/String.h"

void moveCallback(const std_msgs::String::ConstPtr& msg){
	ROS_INFO("I heard: [%s]", msg->data.c_str());

	// RaspiRobot::getInstance()->forwardByTime(10);
}

int main(int argc, char **argv){

	ros::init(argc, argv, "basic_move");

	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("move_chatter", 1000, moveCallback);

	ros::spin();

	// RaspiRobot::init();

	return 0;
}