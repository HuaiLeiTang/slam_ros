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

using namespace std;


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

int main(int argc, char **argv)
{
ros::init(argc, argv, "theseus");
ros::NodeHandle n;
GridMap gmap(15,50,&n);
ros::Publisher thesues_pub = n.advertise<std_msgs::Float32MultiArray>("path", 1000);
std_msgs::Float32MultiArray path;
Node firstNode(0,0);
Node goal(400,200);
firstNode.partOf = true;
RRTs test(LineMapGenerator(),firstNode,500,500);
test.PathPlaning(goal);
test.ExportGraf();
cout<<test.dijkPath.size()<<endl;
for(int i = 0; i < test.dijkPath.size(); i++) {
    cout<<test.dijkPath[i].x<<" "<<test.dijkPath[i].y<<endl;
    path.data.insert(path.data.begin(),test.dijkPath[i].x);
    path.data.insert(path.data.begin(),test.dijkPath[i].y);
}
cout<<"Publish..."<<endl;
gmap.DrawObstacle(test.obstacles);
gmap.DrawCircle(Vec2(2,2),3);
//gmap.DrawArc(200,Vec2(0,2),Vec2(2,0),KNOWN,false);
cout<<"eddig oké vagoyk"<<endl;
gmap.PublishMap();

//thesues_pub.publish(path);
//    ros::spinOnce();
return 0;
}
