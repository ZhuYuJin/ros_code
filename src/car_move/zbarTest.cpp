
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <stdlib.h> 

using namespace std;

void moveCallback(const std_msgs::String::ConstPtr& msg){
	ROS_INFO("I heard: [%s]", msg->data.c_str());

	double z, d, x1, x2, f;
	d = 13.9;
	z = 28.0;
	f = 600.0;

	stringstream ss(msg->data.c_str());
	string temp;
	for(int i = 0; i < 7; i++){
		ss >> temp;
		if(i == 2) x1 = atof(temp.c_str());
		if(i == 3) x2 = atof(temp.c_str());
	}
	z = d * f / (x2-x1);
	// f = z * (x2-x1) / d;
	ROS_INFO("distance: [%f]", z);
}

int main(int argc, char **argv){

	ros::init(argc, argv, "zbarTest");

	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("barcode", 1000, moveCallback);

	ros::spin();

	return 0;
}