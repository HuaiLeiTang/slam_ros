#include "ros/ros.h"
#include <sstream>
#include <iostream>
#include "node.h"
#include "rrts.h"
#include "vec2.h"
#include <fstream>
#include "std_msgs/Float32MultiArray.h"
#include "LineXtraction.h"
#include "gridmap.h"
#include <unistd.h>




using namespace std;
GridMap* gmap;

std::vector<Vec2> lineIntervals;
std::vector<AncientObstacle*> robstacles;
bool newlines = false;
bool newpose = false;
bool targetActive = true; //bool targetActive = false; saját térképezéskor
bool tripEnd = true;
double theta;
bool inGo = false;
Vec2 pose;
bool turn = false;
bool inTurn = false;
bool go = false;
bool firstCilkus = true;
bool noPath = false;
geometry_msgs::Vector3 command;
vector<Vec2> path;
double roundParam;

Vec2 temp;
Node target(-960,-960);


void lines_cb(const std_msgs::Float32MultiArray::ConstPtr& array) {
    lineIntervals.clear();
    robstacles.clear();
    for(std::vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); it = it + 4)
    {
        lineIntervals.push_back(Vec2(*(it)*100,*(it + 1)*100)); // mer hülye vagyok és cm-ben kezdtem irni mident
        lineIntervals.push_back(Vec2(*(it + 2)*100,*(it + 3)*100));
    }
    ofstream output_file;
    ofstream linefile;
    linefile.open("line.txt");
    output_file.open("p_data.txt");
    for(int i = 0; i < lineIntervals.size() - 1; i = i + 2) {
        output_file<<lineIntervals[i]<<endl;
        output_file<<lineIntervals[i + 1]<<endl;
    }
    AncientObstacle* newobs;
    for(int i = 0; i < lineIntervals.size(); i = i + 2) {
        newobs = new StraightObstacle(lineIntervals[i],lineIntervals[i + 1]);
        robstacles.push_back(newobs);
    }
    /*test.SetPose(pose);
    test.PathPlaning(target);
    path.push_back(test.dijkPath[test.dijkPath.size() - 1]);
    path.push_back(test.dijkPath[test.dijkPath.size() - 2]);
    test.ExportGraf();
    test.Reset();*/
    newlines = true;
    return;
}

void pose_cb(geometry_msgs::Vector3 msg) {
    pose.x = msg.x*100;
    pose.y = msg.y*100;
    theta = msg.z;
    newpose = true;
    Vec2 temptarget(target.x,target.y);
    Vec2 temppose(pose.x,pose.y);
    if((Distance(temppose,temptarget) < 30) || (gmap->data[gmap->MapIndex(gmap->Vec2Quantization(target))]) == KNOWN) { // 10 cm sugaru körön belül vagyunk
        targetActive = false;
        cout<<"Reach Traget!"<<endl;
    }
    if(inTurn) {
        go = true;
        inTurn = false;
    }
    if(inGo) {
        inGo = false;
        tripEnd = true;
    }
}

// Main

int main( int argc, char *argv[] )
{

    ros::init(argc, argv, "theseus");
    ros::NodeHandle n;
    GridMap gridmap(42.5,200,&n);
    gmap = &gridmap;
    Astar pathfinder;
    ros::Publisher thesues_pub = n.advertise<std_msgs::Float32MultiArray>("path", 100);
    ros::Subscriber sublines = n.subscribe<std_msgs::Float32MultiArray>("lines_1",100,lines_cb);
    ros::Subscriber subpose = n.subscribe<geometry_msgs::Vector3>("robotPose",100,pose_cb);
    ros::Publisher pubCommand = n.advertise<geometry_msgs::Vector3>("/motionCommand",100);
    //std_msgs::Float32MultiArray path;


    ros::Rate s(10);
    while(ros::ok) {
        if(firstCilkus) {
            if((pubCommand.getNumSubscribers() == 1) && (sublines.getNumPublishers() == 1)) {
                command.x = 1;
                command.y = 0;
                command.z = 0;
                pubCommand.publish(command);
                firstCilkus = false;
            }
        }
        if((newlines == true) && (newpose == true)) {
            cout<<"New Ciklus"<<endl;
            gridmap.SetRobotPose(pose);
            cout<<"Set Robot Pose"<<endl;
            gridmap.DrawObstacle(robstacles);
            cout<<"DrawObstacle"<<endl;
            gridmap.UpgradeKnownGrid(robstacles);
            cout<<"UpgradeKnownGrid"<<endl;
            gridmap.UpgradeTargets(robstacles);
            cout<<"UpgradeTargets"<<endl;
            if(tripEnd) {
                cout<<"New Trip"<<endl;
                tripEnd = false;
                if(!targetActive) { // ide !(!targetActive kell
                    targetActive = true;
                    temp = gridmap.NextGoal();
                    cout<<"New Target: "<<temp<<endl;
                    if(temp.x == 0 && temp.y == 0) {
                        cout<<"No more targets"<<endl;
                        exit(1);
                    }
                    target.x = temp.x;
                    target.y = temp.y;
                }
                pathfinder.Reset();
                cout<<"Path Planning..."<<endl;
                pathfinder.FindPath(pose,target);
                cout<<"Path Planning end!"<<endl;
                path.clear();
                if(pathfinder.vecpath.size() != 0) {
                    path.push_back(pose);
                    path.push_back(pathfinder.vecpath[1]);
                    turn = true; // TODO 5000res rrrt ciklus után uj célpont kérése
                }
                else {
                    noPath = true;
                }
            }
            newlines = false;
            newpose = false;
            gridmap.PublishMap();
            pathfinder.Reset();
            if(noPath) {
                cout<<"No path"<<endl;
                newlines = true;
                newpose = true;
                tripEnd = true;
                targetActive = false;
                noPath = false;
            }
        }
        if(turn) {
            turn = false;
            inTurn = true;
            roundParam = atan2(path[1].y - path[0].y,path[1].x - path[0].x);
            roundParam = roundParam - theta;
            if(roundParam >= PI ) {
                roundParam = -(2*PI - roundParam);
            }
            if(roundParam <= -PI) {
                roundParam = 2*PI + roundParam;
            }
            if(roundParam > 0) {
                command.x = 2;
                command.y = roundParam;
                pubCommand.publish(command);
            }
            else {
                command.x = 3;
                command.y = -roundParam;
                pubCommand.publish(command);
            }
        }
        if(go) {
            //sleep(15);
            go = false;
            inGo = true;
            command.x = 1;
            cout<<"Forward to "<<path[1]<<endl;
            if( (((path[1] - pose).Lenght())/100) > 3 )
                command.y = 3;
            else {
                command.y = ((path[1] - pose).Lenght())/100;
            }
            pubCommand.publish(command);
        }
        s.sleep();
        ros::spinOnce();
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
