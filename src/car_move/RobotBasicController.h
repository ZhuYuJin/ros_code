
#include <stdio.h>
#include <wiringPi.h>
#include <softPwm.h>

#define uchar unsigned char

#define HIGH			1
#define LOW				0

#define LEFT_IN_PIN		7
#define LEFT_OUT_PIN	0
#define RIGHT_IN_PIN	2
#define RIGHT_OUT_PIN	3
#define	LEFT_EN_PWM		1
#define RIGHT_EN_PWM	4

using namespace std;

class RaspiRobot
{
	private:
		static RaspiRobot *instance;
		int direction;
		RaspiRobot();
		void setMotors(uchar leftIn, uchar leftOut, uchar rightIn, uchar rightOut, uchar leftEn, uchar rightEn);

	public:
		static bool init();
		static RaspiRobot *getInstance();
		void stop();
		void forwardBySpeed(int speed);
		void forwardByTimeAndSpeed(float sec, int speed = 100);
		void reverseBySpeed(int speed);
		void reverseByTimeAndSpeed(float sec, int speed = 100);
};

RaspiRobot *RaspiRobot::getInstance()
{
	if(instance==NULL)
		instance=new RaspiRobot();
	return instance;
}

RaspiRobot::RaspiRobot()
{
	
}

RaspiRobot *RaspiRobot::instance=NULL;

bool RaspiRobot::init()
{
	if(wiringPiSetup() < 0){
		printf("setup wiringPi failed !");
		return false;
	}
	
	pinMode(LEFT_IN_PIN, OUTPUT);
	pinMode(LEFT_OUT_PIN, OUTPUT);
	pinMode(RIGHT_IN_PIN, OUTPUT);
	pinMode(RIGHT_OUT_PIN, OUTPUT);
	softPwmCreate(LEFT_EN_PWM, 0, 255);
	softPwmCreate(RIGHT_EN_PWM, 0, 255);
	return true;
}

void setMotors(uchar leftIn, uchar leftOut, uchar rightIn, uchar rightOut, uchar leftEn, uchar rightEn)
{
	digitalWrite(LEFT_IN_PIN, leftIn);
	digitalWrite(LEFT_OUT_PIN, leftOut);
	digitalWrite(RIGHT_IN_PIN, rightIn);
	digitalWrite(RIGHT_OUT_PIN, rightOut);
	softPwmWrite(LEFT_EN_PWM, leftEn);
	softPwmWrite(RIGHT_EN_PWM, rightEn);
}

void RaspiRobot::stop()
{
	setMotors(0,0,0,0,0,0);
}

void forwardBySpeed(int speed)
{
	setMotors(1,0,1,0,speed,speed);
}

void forwardByTimeAndSpeed(float sec, int speed = 100)
{
	setMotors(1,0,1,0,speed,speed);
	if(sec>0)
	{
		delay((int)(sec*1000));
		stop();
	}
}

void reverseBySpeed(int speed)
{
	setMotors(0,1,0,1,speed,speed);
}

void reverseByTimeAndSpeed(float sec, int speed = 100)
{
	setMotors(0,1,0,1,speed,speed);
	if(sec>0)
	{
		delay((int)(sec*1000));
		stop();
	}
}