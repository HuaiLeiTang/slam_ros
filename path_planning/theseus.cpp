#include "ros/ros.h"
#include <sstream>
#include <iostream>
#include "node.h"
#include "rrts.h"
#include "vec2.h"
#include <fstream>
#include "std_msgs/Float32MultiArray.h"

using namespace std;

vector< pair<double,double> >  MapGenerator(void) {
    pair<double,double> seed;
    vector< pair<double,double> > map;
    for(int i = 1; i < 80; i++) {
        seed.second = i*10 - 300;
        seed.first = -200;
        map.push_back(seed);
    }
    for(int i = 1; i < 70; i++) {
        seed.second = i*10 - 500;
        seed.first = 0;
        map.push_back(seed);
    }
    for(int i = 1; i < 80; i++) {
        seed.second = i*10 - 300;
        seed.first = 200;
        map.push_back(seed);
    }
    for(int i = 1; i < 20; i++) {
        seed.second = 0;
        seed.first = 300 + i*10;
        map.push_back(seed);
    }
    for(int i = 1; i < 20; i++) {
        seed.first = -200 - i*10;
        seed.second = -300;
        map.push_back(seed);
    }
    /*seed.first = 80;
    seed.second = 80;
    map.push_back(seed);
    seed.first = -80;
    seed.second = -80;/
    map.push_back(seed);*/
    return map;
}

int main(int argc, char **argv)
{
ros::init(argc, argv, "theseus");
ros::NodeHandle n;
ros::Publisher thesues_pub = n.advertise<std_msgs::Float32MultiArray>("path", 1000);
std_msgs::Float32MultiArray path;
Node firstNode(-400,-200);
Node goal(400,200);
firstNode.partOf = true;
RRTs test(MapGenerator(),firstNode,500,500);
test.PathPlaning(goal);
test.ExportGraf();
cout<<test.dijkPath.size()<<endl;
for(int i = 0; i < test.dijkPath.size(); i++) {
    cout<<test.dijkPath[i].x<<" "<<test.dijkPath[i].y<<endl;
    path.data.insert(path.data.begin(),test.dijkPath[i].x);
    path.data.insert(path.data.begin(),test.dijkPath[i].y);
}
cout<<"Publish..."<<endl;

thesues_pub.publish(path);
    ros::spin();
return 0;
}
