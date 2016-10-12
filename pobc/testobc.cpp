#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <math.h>

#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Vector3.h"
#include "vec2.h"

using namespace std;

vector<Vec2> Arr;
bool msgReceive = false;
bool poseReceive = false;
float motionState = 0;
bool poseRequest = false;
Vec2 pose;
float fi;

#define PI 3.14159


void arrayCallback(const std_msgs::Float32MultiArray::ConstPtr& array) {
    for(std::vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); it = it + 2)
    {
        Arr.push_back(Vec2(*(it+1)/100,*it/100));
        msgReceive = true;
    }
    return;
}

void motionStateCallback(const std_msgs::Float32::ConstPtr& state) {
    motionState = state->data;
    return;
}

void robotPoseCallBack(const geometry_msgs::Vector3::ConstPtr& rec_pose) {
    poseReceive = true;
    pose.x = rec_pose->x;
    pose.y = rec_pose->y;
    fi = rec_pose->z;
    cout<<"calbaaask"<<endl;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "obc");
    ros::NodeHandle n;
    ros::Subscriber sub3 = n.subscribe("path", 100, arrayCallback);
    ros::Subscriber subMotionState = n.subscribe("/motionState",100,motionStateCallback);
    ros::Subscriber subPose = n.subscribe("/robotPose",100,robotPoseCallBack);
    ros::Publisher pubCommand = n.advertise<geometry_msgs::Vector3>("/motionCommand",100);
    ros::Rate rate(10);
    bool motionCommandSended = false;
    bool startMotionCommand = false;
    bool forwardCommand = false;
    bool waitFinishMotion = false;
    bool waitFinishStart = false;
    bool waitForZero = false;
    int commandth = 0;
    double roundParam = 0;
    int counter = 0;
    Vec2 direction;
    geometry_msgs::Vector3 command;
    while(ros::ok) {
        command.x = 5;
        pubCommand.publish(command);
        if(msgReceive) {
            for(int i = 0; i < Arr.size(); i++) {
                cout<<Arr[i].x<<" "<<Arr[i].y<<endl;
            }
            msgReceive = false;
            poseRequest = true;
            cout<<"Path Receive"<<endl;
        }
        if(poseRequest) {
            poseRequest = false;
            command.x = 4;
            command.y = 0;
            command.z = 0;
            pubCommand.publish(command);
        }
        if(poseReceive) {
            poseReceive = false;
            cout<<"Robot pose: "<<pose.x<<" "<<pose.y<<" "<<fi<<endl;
            startMotionCommand = true;
        }
        if(startMotionCommand) {
            cout<<"SendRotateCommand"<<endl;
            startMotionCommand = false;
            waitFinishStart = true;
            roundParam = atan2(Arr[commandth + 1].y - Arr[commandth].y,Arr[commandth + 1].x - Arr[commandth].x);
            roundParam = roundParam - fi;
            cout<<"roundParam - fi = "<<roundParam<<endl;
            if(roundParam >= PI ) {
                roundParam = -(2*PI - roundParam);
            }
            if(roundParam <= -PI) {
                roundParam = 2*PI + roundParam;
            }
            if(roundParam > 0) {
                command.x = 2;
                command.y = roundParam;
                cout<<"Left :"<<command.y<<endl;
                pubCommand.publish(command);
            }
            else {
                command.x = 3;
                command.y = -roundParam;
                pubCommand.publish(command);
                cout<<"Right :"<<command.y<<endl;
            }
        }
        if(waitFinishStart) {
            if(motionState == 1) {
                waitForZero = true;
            }
            if(waitForZero && (motionState == 0)) {
                waitForZero = false;
                forwardCommand = true;
                waitFinishStart = false;
                cout<<"EnableNextCommand"<<endl;
            }
        }
        if(forwardCommand) {
            forwardCommand = false;
            waitFinishMotion = true;
            command.x = 1;
            command.y = (Arr[commandth + 1] - pose).Lenght();
            cout<<"Forward :"<<command.y<<endl;
            commandth++;
            pubCommand.publish(command);
        }
        if(waitFinishMotion) {
            if(motionState == 1) {
                waitForZero = true;
            }
            if(waitForZero && (motionState == 0)) {
                waitFinishMotion == false;
                poseRequest = true;
                waitForZero = false;
            }
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

