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
geometry_msgs::Vector3 command;
vector<Vec2> path;
double roundParam;

Vec2 temp;
Node target(-700,-700);

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
    for(std::vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); it = it + 4)
    {
        lineIntervals.push_back(Vec2(*(it)*100,*(it + 1)*100)); // mer hülye vagyok és cm-ben kezdtem irni mident
        lineIntervals.push_back(Vec2(*(it + 2)*100,*(it + 3)*100));
    }
    AncientObstacle* newobs;
    for(int i = 0; i < lineIntervals.size(); i = i + 2) {
        newobs = new StraightObstacle(lineIntervals[i],lineIntervals[i + 1]);
        robstacles.push_back(newobs);
    }
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
    }
}

int main(int argc, char **argv)
{
ros::init(argc, argv, "theseus");
ros::NodeHandle n;
GridMap gmap(16,200,&n);
ros::Publisher thesues_pub = n.advertise<std_msgs::Float32MultiArray>("path", 1000);
ros::Subscriber sublines = n.subscribe<std_msgs::Float32MultiArray>("lines",100,lines_cb);
ros::Subscriber subpose = n.subscribe<geometry_msgs::Vector3>("robotPose",100,pose_cb);
ros::Publisher pubCommand = n.advertise<geometry_msgs::Vector3>("/motionCommand",100);
//std_msgs::Float32MultiArray path;
Node firstNode(-400,-200);
Node goal(500,500);
firstNode.partOf = true;
RRTs test(LineMapGenerator(),firstNode,1000,1000);
test.obstacles.clear();
test.Reset();

ros::Rate s(10);
while(ros::ok) {
    if((newlines == true) && (newpose == true) && (inGo == false)) {
        cout<<"Start wait sensor"<<endl;
       // sleep(8);
        cout<<"End wait sensor"<<endl;
        gmap.SetRobotPose(pose);
        cout<<"Set Pose for Gmap: "<<pose<<endl;
        gmap.DrawObstacle(robstacles);
        gmap.UpgradeKnownGrid(robstacles);
        gmap.UpgradeTargets(robstacles);
        test.AddObstacles(robstacles);
        test.Reset();
        if(tripEnd) {
            tripEnd = false;
            test.Reset();
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
            cout<<"Start path planing"<<endl;
            test.PathPlaning(target);
            path.push_back(test.dijkPath[test.dijkPath.size() - 1]);
            path.push_back(test.dijkPath[test.dijkPath.size() - 2]);
            cout<<"New path"<<endl;
            test.ExportGraf();
            cout<<"Exportgraf"<<endl;
            turn = true;
        }
      //  gmap.PublishMap();
        newlines = false;
        newpose = false;
        robstacles.clear();
        cout<<"drawing"<<endl;
    }
    if(turn) {
        turn = false;
        inTurn = true;
        roundParam = atan2(path[1].y - path[0].y,path[1].x - path[0].x);
        roundParam = roundParam - theta;
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
    if(go) {
        //sleep(15);
        go = false;
        inGo = true;
        tripEnd = true;
        command.x = 1;
        if( (((path[1] - pose).Lenght())/100) > 3.5 )
            command.y = 3.5;
        else {
            command.y = ((path[1] - pose).Lenght())/100;
        }
        cout<<"Forward :"<<command.y<<endl;
        pubCommand.publish(command);
        path.clear();

    }
    s.sleep();
    ros::spinOnce();
}

/*while(ros::ok) {
    if((newlines == true) && (newpose == true)) {
        gmap.SetRobotPose(pose);
        gmap.DrawObstacle(robstacles);
        gmap.UpgradeKnownGrid(robstacles);
        gmap.UpgradeTargets(robstacles);
        gmap.PublishMap();
    }

    s.sleep();
    ros::spinOnce();
}*/


return 0;
}
