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
bool targetActive = false;
bool tripEnd = true;
double theta;
bool inGo = false;
Vec2 pose;
bool turn = false;
bool inTurn = false;
bool go = false;
bool firstCilkus = true;
geometry_msgs::Vector3 command;
vector<Vec2> path;
double roundParam;

Vec2 temp;
Node target(-700,-700);

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
    cout<<"test obs elotte: "<<test.obstacles.size()<<endl;
    test.AddObstacles(robstacles);
    cout<<"test obs utana: "<<test.obstacles.size()<<endl;
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
  /*  if(Distance(temppose,temptarget) < 50) { // 10 cm sugaru körön belül vagyunk
        targetActive = false;
    }*/
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
GridMap gmap(22,200,&n);
ros::Publisher thesues_pub = n.advertise<std_msgs::Float32MultiArray>("path", 1000);
ros::Subscriber sublines = n.subscribe<std_msgs::Float32MultiArray>("lines",100,lines_cb);
ros::Subscriber subpose = n.subscribe<geometry_msgs::Vector3>("robotPose",100,pose_cb);
ros::Publisher pubCommand = n.advertise<geometry_msgs::Vector3>("/motionCommand",100);
//std_msgs::Float32MultiArray path;


ros::Rate s(10);
while(ros::ok) {
    if(firstCilkus) {
        if(pubCommand.getNumSubscribers() == 1) {
            command.x = 1;
            command.y = 0;
            command.z = 0;
            pubCommand.publish(command);
            firstCilkus = false;
        }
    }
    if((newlines == true) && (newpose == true)) {
        gmap.SetRobotPose(pose);
        gmap.DrawObstacle(robstacles);
        gmap.UpgradeKnownGrid(robstacles);
        gmap.UpgradeTargets(robstacles);
      /*  test.SetPose(pose);
        test.PathPlaning(target);
        test.ExportGraf();*/
        if(tripEnd) {
            tripEnd = false;
            if(false) { // ide !(!targetActive kell
                targetActive = true;
                temp = gmap.NextGoal();
                cout<<"New Target----------- "<<temp<<endl;
                cout<<"--------------------------"<<endl;
                if(temp.x == 0 && temp.y == 0) {
                    cout<<"No more targets"<<endl;
                    exit(1);
                }
                target.x = temp.x;
                target.y = temp.y;
            }
            test.SetPose(pose);
            test.PathPlaning(target);
            test.ExportGraf();
            path.clear();
            path.push_back(test.dijkPath[test.dijkPath.size() - 1]);
            path.push_back(test.dijkPath[test.dijkPath.size() - 2]);
            turn = true;
        }
        test.Reset();
        newlines = false;
        newpose = false;
        gmap.PublishMap();
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


return 0;
}
