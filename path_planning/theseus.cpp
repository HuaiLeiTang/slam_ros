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

std::vector<Vec2> lineIntervals;
std::vector<AncientObstacle*> robstacles;
bool newlines = false;
bool newpose = false;
bool targetActive = true; //targetActive = false;
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
Node target(-970,-970);

RRTs test(1000,1000);

vector< AncientObstacle* >  LineMapGenerator(void) {
    vector<double> alfa;
    vector<double> r;
    ifstream iFile("/home/mono/build-lineTry-Desktop_Qt_5_7_0_GCC_64bit-Default/alfa.txt");
    std::string input = "";
    if( iFile.is_open() )
    {
        while( iFile.good() )
        {
            std::getline( iFile, input, '\n' );

            std::stringstream ss( input );
            double myDouble = 0;

            if( ss >> myDouble ) {
                alfa.push_back(myDouble);
            }
            else {

            }
        }

        iFile.close();
    }
    else
        std::cout << "Error opening file...\n";

    ifstream File("/home/mono/build-lineTry-Desktop_Qt_5_7_0_GCC_64bit-Default/r.txt");
    input = "";
    if( File.is_open() )
    {
        while( File.good() )
        {
            std::getline( File, input, '\n' );

            std::stringstream ss( input );
            double myDouble = 0;

            if( ss >> myDouble ) {
                r.push_back(myDouble);
            }
            else {

            }
        }

        File.close();
    }
    else
        std::cout << "Error opening file...\n";
    alfa.erase(alfa.end() - 1);
    r.erase(r.end() - 1);
    polar_point temp;
    vector<polar_point> data;
    default_random_engine general;
    normal_distribution<double> gauss_nois(0,0.01);
    vector<polar_point> newdata;
    for(int i = 0; i < alfa.size(); i++) {
        temp.alfa = alfa[i];
        temp.r = r[i];// +  gauss_nois(general);
        temp.variance = 0.03;
        temp.weight = 1/(0.03*0.03);
        temp.alfaVariance = pow(PI/180,2)/12;
        data.push_back(temp);

    }
    //rotate(data.begin(),data.begin()+10,data.end());
    vector<line> lines;
    lines = LineExtraction(data);
    AncientObstacle* newobs;
    vector<AncientObstacle*> obstacles;
    for(int i = 0; i < lines.size(); i++) {
        newobs = new StraightObstacle(lines[i].lineInterval[0],lines[i].lineInterval[1]);
        obstacles.push_back(newobs);
    }
    return obstacles;
}

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
    test.AddObstacles(robstacles);
    for(int i = 0; i < test.obstacles.size(); i++) {
        if(test.obstacles[i]->type == lineObstacle) {
            test.obstacles[i]->Draw(linefile);
        }
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
    if(Distance(temppose,temptarget) < 30) { // 10 cm sugaru körön belül vagyunk
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

int main(int argc, char **argv)
{
ros::init(argc, argv, "theseus");
ros::NodeHandle n;
GridMap gmap(37,200,&n);
ros::Publisher thesues_pub = n.advertise<std_msgs::Float32MultiArray>("path", 1000);
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
        gmap.SetRobotPose(pose);
        cout<<"Set Robot Pose"<<endl;
        gmap.DrawObstacle(robstacles);
        cout<<"DrawObstacle"<<endl;
        gmap.UpgradeKnownGrid(robstacles);
        cout<<"UpgradeKnownGrid"<<endl;
        gmap.UpgradeTargets(robstacles);
        cout<<"UpgradeTargets"<<endl;
        if(tripEnd) {
            cout<<"New Trip"<<endl;
            tripEnd = false;
            if(!targetActive) { // ide !(!targetActive kell
                targetActive = true;
              /*  temp = gmap.NextGoal();
                cout<<"New Target: "<<temp<<endl;
                if(temp.x == 0 && temp.y == 0) {
                    cout<<"No more targets"<<endl;
                    exit(1);
                }
                target.x = temp.x;
                target.y = temp.y;*/
            }
            test.SetPose(pose);
            cout<<"Path Planning..."<<endl;
            test.PathPlaning(target);
            cout<<"Path Planning end!"<<endl;
            path.clear();
            if(test.dijkPath.size() != 0) {
                test.ExportGraf();
                path.push_back(test.dijkPath[test.dijkPath.size() - 1]);
                path.push_back(test.dijkPath[test.dijkPath.size() - 2]);
                turn = true; // TODO 5000res rrrt ciklus után uj célpont kérése
            }
            else {
                noPath = true;
            }
        }
        test.Reset();
        newlines = false;
        newpose = false;
        gmap.PublishMap();
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
        if( (((path[1] - pose).Lenght())/100) > 1.5 )
            command.y = 1.5;
        else {
            command.y = ((path[1] - pose).Lenght())/100;
        }
        pubCommand.publish(command);
    }
    s.sleep();
    ros::spinOnce();
}


return 0;
}
