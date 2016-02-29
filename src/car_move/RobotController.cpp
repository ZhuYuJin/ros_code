#include <wiringPi.h>
#include <cstdio>
#include <sys/time.h>
#include "RobotController.h"

#define LEFT_GO_PIN 0
#define LEFT_DIR_PIN 7
#define RIGHT_GO_PIN 12
#define RIGHT_DIR_PIN 6
#define SW1_PIN 14
#define SW2_PIN 13
#define LED1_PIN 11
#define LED2_PIN 10
#define OC1_PIN 3
#define OC2_PIN 2
#define TRIGGER_PIN 1
#define ECHO_PIN 4

using namespace std;

RaspiRobot *RaspiRobot::instance=NULL;

bool RaspiRobot::init()
{
	if(wiringPiSetup()<0)
		return false;
	
	pinMode(LEFT_GO_PIN,OUTPUT);
	pinMode(LEFT_DIR_PIN,OUTPUT);
	pinMode(RIGHT_GO_PIN,OUTPUT);
	pinMode(RIGHT_DIR_PIN,OUTPUT);
	pinMode(SW1_PIN,INPUT);
	pinMode(SW2_PIN,INPUT);
	pinMode(LED1_PIN,OUTPUT);
	pinMode(LED2_PIN,OUTPUT);
	pinMode(OC1_PIN,OUTPUT);
	pinMode(OC2_PIN,OUTPUT);
	pinMode(TRIGGER_PIN,OUTPUT);
	pinMode(ECHO_PIN,INPUT);
	return true;
}

RaspiRobot *RaspiRobot::getInstance()
{
	if(instance==NULL)
		instance=new RaspiRobot();
	return instance;
}

RaspiRobot::RaspiRobot()
{
	direction=90;
}

void RaspiRobot::setMotors(unsigned char leftGo,unsigned char leftDir,unsigned char rightGo,unsigned char rightDir)
{
	digitalWrite(LEFT_GO_PIN,leftGo);
	digitalWrite(LEFT_DIR_PIN,leftDir);
	digitalWrite(RIGHT_GO_PIN,rightGo);
	digitalWrite(RIGHT_DIR_PIN,rightDir);
}

void RaspiRobot::forwardByTime(float sec=0)
{
	setMotors(1,0,1,1);
	if(sec>0)
	{
		delay((int)(sec*1000));
		stop();
	}
}

void RaspiRobot::reverseByTime(float sec=0)
{
	setMotors(1,1,1,0);
	if(sec>0)
	{
		delay((int)(sec*1000));
		stop();
	}
}

void RaspiRobot::forwardByDistance(int centimeter=0)
{
	//TODO
}

void RaspiRobot::reverseByDistance(int centimeter=0)
{
	//TODO
}

void RaspiRobot::stop()
{
	setMotors(0,0,0,0);
}

void RaspiRobot::turnLeftByTime(float sec=0)
{
	setMotors(1,1,1,1);
	if(sec>0)
	{
		delay((int)(sec*1000));
		stop();
	}
}

void RaspiRobot::turnRightByTime(float sec=0)
{
	setMotors(1,0,1,0);
	if(sec>0)
	{
		delay((int)(sec*1000));
		stop();
	}
}

void RaspiRobot::turnLeftByDegree(int degree=0)
{
	//TODO
}

void RaspiRobot::turnRightByDegree(int degree=0)
{
	//TODO
}

unsigned char RaspiRobot::getSwitch1()
{
	return (unsigned char)digitalRead(SW1_PIN);
}

unsigned char RaspiRobot::getSiwtch2()
{
	return (unsigned char)digitalRead(SW2_PIN);
}

void RaspiRobot::setLED1(unsigned char state)
{
	digitalWrite(LED1_PIN,state);
}

void RaspiRobot::setLED2(unsigned char state)
{
	digitalWrite(LED2_PIN,state);
}

void RaspiRobot::setOC1(unsigned char state)
{
	digitalWrite(OC1_PIN,state);
}

void RaspiRobot::setOC2(unsigned char state)
{
	digitalWrite(OC2_PIN,state);
}

float RaspiRobot::getDistance(float minDistance,float maxDistance,int count,int maxLoop)
{
	int successfulCount=0;
	float totalDistance=0;
	struct timeval t1,t2;
	
	while(maxLoop--)
	{
		digitalWrite(TRIGGER_PIN,LOW);
		delayMicroseconds(2);
		digitalWrite(TRIGGER_PIN,HIGH);
		delayMicroseconds(10);
		digitalWrite(TRIGGER_PIN,LOW);
		
		int countdown=2000000;
		while(digitalRead(ECHO_PIN)!=HIGH&&countdown>0)
		{
			countdown--;
			//Without pulseIn() function on Arduino, so we can only wait here
		}
		if(countdown==0)
			continue;
		gettimeofday(&t1,NULL);

		countdown=5000000;
		while(digitalRead(ECHO_PIN)!=LOW&&countdown>0)
		{
			countdown--;
			//Without pulseIn() function on Arduino, so we can only wait here
		}
		if(countdown==0)
			continue;
		gettimeofday(&t2,NULL);
		
		int startMicrosecond=t1.tv_sec*1000000+t1.tv_usec;
		int endMicrosecond=t2.tv_sec*1000000+t2.tv_usec;
		float distance=(endMicrosecond-startMicrosecond)/58.8235;
		
		if(distance<minDistance||distance>maxDistance)
			continue;
		successfulCount++;
		totalDistance+=distance;
		
		if(successfulCount==count)
			return totalDistance/count;
	}
	return -1;
}

float RaspiRobot::getDistance()
{
	return getDistance(2.0,450.0,10,20);
}