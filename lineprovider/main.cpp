#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <unistd.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/Vector3.h"
#include "LineXtraction.h"
#include <fstream>

using namespace std;

std::vector<polar_point> points;
std::vector<line> lines;
std_msgs::Float32MultiArray lineIntervals;
double theta;
Vec2 pose;
bool newlines = false;
bool newpose = false;

void mapping_cb(std_msgs::Float32MultiArray msg){
    ofstream output_file;
    output_file.open("receive.txt");
    points.clear();
    lines.clear();
    polar_point temp;
    cout<<"start line asd"<<endl;
    std::default_random_engine generator;
    std::normal_distribution<double> distribution(0.0,0.1);
    for(int i = 0; i < msg.layout.dim[0].size; i+=2){
        points.push_back(temp);
        points[points.size()-1].alfa = msg.data[i+1] - M_PI;
        points[points.size()-1].r = msg.data[i];// + distribution(generator) ;
        points[points.size()-1].variance = 0.05;
        output_file<<points[points.size()-1].r<<" "<<points[points.size()-1].alfa + M_PI<<endl;
    }
    lines = LineExtraction(points);
    for(auto &lin : lines){
        lin.WriteCov();
    }
    newlines = true;
}

void pose_cb(geometry_msgs::Transform msg) {
    pose.x = msg.translation.x;
    pose.y = msg.translation.y;
    theta = msg.translation.z;
    newpose = true;
}

void Transform(){
    Vec2 ex(cos(theta),sin(theta));
    double temp = theta + PI/2;
    temp = temp > M_PI ? temp-2.0*M_PI : temp;
    Vec2 ey(cos(temp),sin(temp));
    Vec2 first;
    Vec2 end;
    Point tempPoint;
    for(int i = 0; i < lines.size(); i++) {
        tempPoint = polar2descart(lines[i].lineInterval[0]);
        first.x = tempPoint.x;
        first.y = tempPoint.y;
        tempPoint = polar2descart(lines[i].lineInterval[1]);
        end.x = tempPoint.x;
        end.y = tempPoint.y;
        first = ex*first.x + ey*first.y;
        end = ex*end.x + ey*end.y;
        first = first + pose;
        end = end + pose;
        lineIntervals.data.push_back(first.x);
        lineIntervals.data.push_back(first.y);
        lineIntervals.data.push_back(end.x);
        lineIntervals.data.push_back(end.y);
    }
}

int main(int argc,char* argv[])
{
    int argc2 = 0;
    char** argv2 = NULL;
    ros::init(argc2, argv2, "linesprovider");
    ros::NodeHandle nh;
    ros::Subscriber subMapping = nh.subscribe<std_msgs::Float32MultiArray>("mappingPoints", 10, &mapping_cb);
    ros::Subscriber subPose = nh.subscribe<geometry_msgs::Transform>("robotPosition",10,&pose_cb);
    ros::Publisher publines = nh.advertise<std_msgs::Float32MultiArray>("lines_1",100);
    ros::Rate r(10);
    while(ros::ok) {
        if( (newlines == true) && (newpose == true) ) {
            if(true) {
                newlines = false;
                newpose = false;
                Transform();
                publines.publish(lineIntervals);
                cout<<"publish: "<<lineIntervals.data.size()<<endl;
                lineIntervals.data.clear();
            }
        }
        ros::spinOnce();
        r.sleep();
    }
    return 0;

}
