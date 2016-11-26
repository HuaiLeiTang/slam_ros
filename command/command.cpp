/*
 * main.cpp
 *
 *  Created on: Sep 8, 2016
 *      Author: mono
 */

#include <iostream>
#include <unistd.h>
#include "uart.h"
#include <string.h> // memcpy
#include <stdlib.h> //realloc
#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"

using namespace std;

#define FORWARD 1
#define LEFT 2
#define RIGHT 3
#define PI 3.14159265

double command;
double param;
Pose pose;
geometry_msgs::Vector3 sendPose;
bool newpose = false;

void command_cb(geometry_msgs::Vector3 msg) {
    command = msg.x;
    if( command == FORWARD) {
        param = msg.y*100;
    }
    else {
        param = msg.y*180/PI;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "command");
    ros::NodeHandle n;
    ros::Publisher realPosePub = n.advertise<geometry_msgs::Vector3>("realRoboPose",100);
    ros::Subscriber commandSub = n.subscribe<geometry_msgs::Vector3>("/motionCommand",100,command_cb);
	CompCom rp6;
    ros::Rate s(10);
    Pose temp;
	time_t startTime = time(0);
	double byte;
    while(ros::ok()){
        switch ((int)command) {
            case FORWARD:
                rp6.SendMotionCommand(param,'F');
                cout<<"Send Forward command"<<endl;
                while(rp6.isMove)
                {
                    temp = rp6.SendPoseRequest();
                    pose = temp.isReal ? temp : pose;
                    sleep(1);
                };
                newpose = true;
                break;
            case LEFT:
                    rp6.SendMotionCommand(param,'L');
                    cout<<"Send Left command"<<endl;
                    while(rp6.isMove) {
                        temp = rp6.SendPoseRequest();
                        pose = temp.isReal ? temp : pose;
                        sleep(1);
                    };
                    newpose = true;
                break;
            case RIGHT:
                rp6.SendMotionCommand(param,'R');
                cout<<"Send Right command"<<endl;
                while(rp6.isMove) {
                    temp = rp6.SendPoseRequest();
                    pose = temp.isReal ? temp : pose;
                    sleep(1);
                };
                newpose = true;
                break;
            default:
                break;
        }
        command = 0;
        s.sleep();
        if(newpose) {
            newpose = false;
            sendPose.x = pose.x/100;
            sendPose.y = pose.y/100;
            sendPose.z = pose.theta;
            cout<<"Dead Reck x "<<sendPose.x<<" y "<<sendPose.y<<" theta "<<sendPose.z<<endl;
            realPosePub.publish(sendPose);
        }
        ros::spinOnce();
	}
}


