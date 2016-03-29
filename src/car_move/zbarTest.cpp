
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <stdlib.h> 
#include <math.h>

#define pi 3.14159262653589793238462643383279

using namespace std;

double ang_r = 75, ang_c = 45; //a->maximum horizontal angle    b->maximum vertical angle
int row, col; //row->maximum horizontal pixel number    col->maximum vertical pixel number
int xc1, yc1, xc2, yc2;	//positions of feature points in the picture(described by pixel number)

double z, d; //d->edge of barcode	z->distance of barcode(to be solved)

void getDistance(int x_mid, int y_mid, int x1, int y1, int x2, int y2, int x3, int y3, int x4, int y4, double distance){
	row = 640;
	col = 480;
	z = 0.0;
	d = distance; //distance between middle point and top-left point

	double k_mid, p_mid, k1, p1, k2, p2, k3, p3, k4, p4;
	xc1 = x_mid;
	yc1 = y_mid;
	k_mid = (2*xc1-row) * tan(double(ang_r)/double(360)*pi) / row;
	p_mid = (2*yc1-col) * tan(double(ang_c)/double(360)*pi) / col;

	xc2 = x1;
	yc2 = y1;
	k1 = (2*xc2-row) * tan(double(ang_r)/double(360)*pi) / row;
	p1 = (2*yc2-col) * tan(double(ang_c)/double(360)*pi) / col;
	z += sqrt(pow(d,2) / (pow((k_mid-k1),2) + pow((p_mid-p1),2)));

	xc2 = x2;
	yc2 = y2;
	k2 = (2*xc2-row) * tan(double(ang_r)/double(360)*pi) / row;
	p2 = (2*yc2-col) * tan(double(ang_c)/double(360)*pi) / col;
	z += sqrt(pow(d,2) / (pow((k_mid-k2),2) + pow((p_mid-p2),2)));

	xc2 = x3;
	yc2 = y3;
	k3 = (2*xc2-row) * tan(double(ang_r)/double(360)*pi) / row;
	p3 = (2*yc2-col) * tan(double(ang_c)/double(360)*pi) / col;
	z += sqrt(pow(d,2) / (pow((k_mid-k3),2) + pow((p_mid-p3),2)));

	xc2 = x4;
	yc2 = y4;
	k4 = (2*xc2-row) * tan(double(ang_r)/double(360)*pi) / row;
	p4 = (2*yc2-col) * tan(double(ang_c)/double(360)*pi) / col;
	z += sqrt(pow(d,2) / (pow((k_mid-k4),2) + pow((p_mid-p4),2)));

	z = z/4;
}

void moveCallback(const std_msgs::String::ConstPtr& msg){
	ROS_INFO("I heard: [%s]", msg->data.c_str());

	double mid_x, mid_y, x_min, x_max, y_min, y_max, x_1, y_1, x_2, y_2, x_3, y_3, x_4, y_4; //infos of barcode
	string str; // message of barcode

	stringstream ss(msg->data.c_str());
	string temp;
	for(int i = 0; i < 15; i++){
		ss >> temp;
		if(i == 0) mid_x = atof(temp.c_str());
		if(i == 1) mid_y = atof(temp.c_str());
		if(i == 2) x_min = atof(temp.c_str());
		if(i == 3) x_max = atof(temp.c_str());
		if(i == 4) y_min = atof(temp.c_str());
		if(i == 5) y_max = atof(temp.c_str());
		if(i == 6) x_1 = atof(temp.c_str());
		if(i == 7) y_1 = atof(temp.c_str());
		if(i == 8) x_2 = atof(temp.c_str());
		if(i == 9) y_2 = atof(temp.c_str());
		if(i == 10) x_3 = atof(temp.c_str());
		if(i == 11) y_3 = atof(temp.c_str());
		if(i == 12) x_4 = atof(temp.c_str());
		if(i == 13) y_4 = atof(temp.c_str());
		if(i == 14) str = temp.c_str();
	}
	
	getDistance(mid_x, mid_y, x_1, y_1, x_2, y_2, x_3, y_3, x_4, y_4, 9.825);

	ROS_INFO("distance: [%f]", z);
}

int main(int argc, char **argv){

	ros::init(argc, argv, "zbarTest");

	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("barcode", 1000, moveCallback);

	ros::spin();

	return 0;
}