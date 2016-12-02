#include <stdio.h>
//#include <iostream>
#include <wiringPi.h>
#include <mcp3004.h>
#include <unistd.h>
#include <ros/ros.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"

#define BASE 100
#define SPI_CHAN 0

bool run = true;
bool forward = true;

void run_cb(std_msgs::Bool msg){
	if(msg.data){
		run = true;
	}
}

int main(int argc, char** argv){
	
	//init ROS
	int argc2 = 0;
	char** argv2 = NULL;
	ros::init(argc2, argv2, "slam");

	//init PWM and SPI
	wiringPiSetup();
	mcp3004Setup(BASE, SPI_CHAN);
	pinMode(1, PWM_OUTPUT);
	pwmSetMode(PWM_MODE_MS);
	pwmSetClock(384);
	pwmSetRange(1000);
	
	ros::NodeHandle nh;
    ros::Publisher pubPoints = nh.advertise<std_msgs::Float32MultiArray>("mappingPoints",100);
    ros::Rate r(1);

    ros::Subscriber subRun = nh.subscribe<std_msgs::Bool>("runSensor", 100, &run_cb);

    usleep(1000000); 
	std_msgs::Float32MultiArray msg;


	while(ros::ok()){
		if(run){
			msg.data.clear();
			msg.data.reserve(400);
			msg.data.resize(400);
			if(forward){
				for(int i = 0; i < 50; ++i){
					pwmWrite(1, 50+i*1.25);
					usleep(50000);
					for(int j = 0; j < 4; ++j){
						float tmp = analogRead(BASE+j)/1024.0*2.8;
						float dist = 16.2537*tmp*tmp*tmp*tmp - 129.893*tmp*tmp*tmp + 382.268*tmp*tmp - 512.611*tmp + 306.439;
						dist = (dist+6.0)/100.0; 
						msg.data[i*2+j*100] = (i/50.0*M_PI/2.0 + j*M_PI/2.0);
						if(dist < 1.8){
							msg.data[i*2+j*100+1] = dist;
						}else{
							msg.data[i*2+j*100+1] = 0.f;
						}
					}
				}
				forward=false;
			}else{
				for(int i = 49; i > -1; --i){
					pwmWrite(1, 50+i*1.25);
					usleep(50000);
					for(int j = 0; j < 4; ++j){
						float tmp = analogRead(BASE+j)/1024.0*2.8;
						float dist = 16.2537*tmp*tmp*tmp*tmp - 129.893*tmp*tmp*tmp + 382.268*tmp*tmp - 512.611*tmp + 306.439;
						dist = (dist+6.0)/100.0; 
						msg.data[i*2+j*100] = (i/50.0*M_PI/2.0 + j*M_PI/2.0);
						if(dist < 1.8){
							msg.data[i*2+j*100+1] = dist;
						}else{
							msg.data[i*2+j*100+1] = 0.f;
						}
					}
				}
				forward=true;
			}
			pubPoints.publish(msg);
			run = false;
			printf("measurement finished");

		}
		//int val = analogRead(BASE);
		//printf("hoi: %i \n", val);
		//std::cout << "measurement finished";
		//text output can't seem to find ssh terminal...

		ros::spinOnce();
        r.sleep();
    }
    subRun.shutdown();
    ros::shutdown();
    return 0;
}
