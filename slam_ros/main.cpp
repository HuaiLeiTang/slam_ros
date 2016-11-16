/*Copyright (C) 2016  Adrián Varga, Menyhart Radó Dávid

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.*/

#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <unistd.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/Vector3.h"
#include "Robot.h"
//#include "Vrep.h"

bool update = true;
float rot[2];
std::vector<polar_point> points;
std::vector<line> lines;

void mapping_cb(std_msgs::Float32MultiArray msg){
    points.clear();
    lines.clear();
    polar_point temp;
    std::cout << "\n\npoint count:" << msg.layout.dim[0].size/2;

    //saving points with gaussian noise
    std::default_random_engine generator;
    std::normal_distribution<double> distribution(0.0,0.004);        //0.4cm standard deviation
    for(int i = 0; i < msg.layout.dim[0].size; i+=2){
        points.push_back(temp);
        points[points.size()-1].alfa = msg.data[i+1] - M_PI;
        //points[points.size()-1].alfa = msg.data[i+1] > M_PI ? msg.data[i+1]-2.0*M_PI : msg.data[i+1];
        points[points.size()-1].r = msg.data[i] + distribution(generator);
        points[points.size()-1].variance = 0.04;
        //points[points.size()-1].weight = 1;
        //std::cout << "\npoint: " << points[points.size()-1].alfa << " " << points[points.size()-1].r;
    }
    std::cout << "\nstarting line extraction";
    lines = LineExtraction(points);
    for(auto &lin : lines){
        lin.alfa += M_PI;
        lin.alfa = lin.alfa > M_PI ? lin.alfa-2.0*M_PI : lin.alfa;
    }
    std::cout << "\nline extraction finished";
    std::cout << "\nresults: ";
    for(int i = 0; i < lines.size(); ++i){
        std::cout << "\n" << lines[i].alfa << " " << lines[i].r;
        lines[i].WriteCov();
    }
    if(lines.empty()){
        std::cout << "\nno lines found";
    }
    std::cout << std::endl;
}
void encoderUpdate_cb(geometry_msgs::Vector3 msg){
    rot[0] = msg.x;
    rot[1] = msg.y;
    update = true;
}

int main(int argc,char* argv[])
{
    
	int argc2 = 0;
	char** argv2 = NULL;
	ros::init(argc2, argv2, "slam");

    Robot* rover = new Robot(0, 0, 0);
    lines.reserve(20);
    points.reserve(250);
    /*Vrep* vrep = new Vrep(argc, argv);

    if(vrep->error != NO_ERROR)
    {
        return 1;
    }
    vrep->loop(rover);
    delete vrep;*/
    /*lines.push_back(line(0.0, 4.0));
    lines.push_back(line(M_PI/M_PI*182.0, 6.2));
    rot[0] = 10*M_PI; rot[1] = 10*M_PI;
    rover->localize(rot, lines);
    for(int i = 0; i < 9; ++i){
        std::cout << " " << rover->P_t0[i] << std::endl;
    }*/

    ros::NodeHandle nh;
    ros::Publisher pubCov = nh.advertise<geometry_msgs::Transform>("robotPosition", 100);
    ros::Publisher publines = nh.advertise<std_msgs::Float32MultiArray>("lines",100);
    ros::Rate r(10);

    ros::Subscriber subEncoder = nh.subscribe<geometry_msgs::Vector3>("encoderPosition", 100, &encoderUpdate_cb);
    ros::Subscriber subMapping = nh.subscribe<std_msgs::Float32MultiArray>("mappingPoints", 10, &mapping_cb);

    usleep(1000000);        //the nodes require time to connect internally (otherwise the "publish" is lost)

    geometry_msgs::Transform msg;

    while(ros::ok()){

        /*int c = getchar();
        if(c == 119){
            std::cout << "FORWARD, MARCH!";
        }*/
        if(update){
            update = false;
            rover->localize(rot, lines);
            lines.clear();

            msg.translation.x = rover->xPos;
            msg.translation.y = rover->yPos;
            msg.translation.z = rover->thetaPos;

            float axii[2];
            float angle;
            if(rover->getEllipse(axii, angle)){
                std::cout << "\n majoraxis: " << axii[1];
                std::cout << "\n minoraxis: " << axii[0];
                std::cout << "\n angle: " << angle;
                std::cout << "\n";
            }else{
                std::cout <<"main: An error occured while computing the eigenvalues and vectors";
            }
            msg.rotation.x = axii[1];
            //msg.rotation.x = 1.0;
            msg.rotation.y = axii[0];
            //msg.rotation.y = 1.0;
            msg.rotation.z = angle;
            pubCov.publish(msg);

            //Publishing line endpoints
            publines.publish(rover->lineIntervals);
            std::cout<<"\n Num Lines: "<< rover->lineIntervals.data.size()/4 <<endl;
            rover->lineIntervals.data.clear();

        }
        ros::spinOnce();
        r.sleep();
    }
    subEncoder.shutdown();
    subMapping.shutdown();
    ros::shutdown();
    return 0;

}


//double fi[] = {-M_PI, M_PI};
//double xi[] = {0, 0, 0};
//Forward Differential Kinematics based on fi
/*rover->forwardKin(fi, xi);
printf("\n Robot Velocity Vector: \n");
int i;
for (i = 0; i < 3; ++i){
        printf ("%g ", xi[i]);
        if(i%1 == 0){
            printf("\n");
            }
}*/

//Inverse Differentail Kinematics based on xi
//xi[0] = -3.816; xi[1] = 0; xi[2] = 4.594;
/*rover->inverseKin(xi, fi);
printf("\n Angular Velocites Vector: \n");
for (i = 0; i < 2; ++i){
        printf ("%g ", fi[i]);
        if(i%1 == 0){
            printf("\n");
            }
}*/


/*
//just a few things for reference
    //double x = 5.0;
    //double y = gsl_sf_bessel_J0 (x);
    //printf ("J0(%g) = %.18e\n", x, y);

    //gsl_matrix* m1 = gsl_matrix_alloc(4, 3);
    //gsl_matrix* m2 = gsl_matrix_alloc(3, 3);
    //gsl_matrix* m3 = gsl_matrix_alloc(4, 4);
    //gsl_matrix_set (m1, 0, 0, 2); gsl_matrix_set (m1, 0, 1, 0); gsl_matrix_set (m1, 0, 2, 2);
    //gsl_matrix_set (m1, 1, 0, 0); gsl_matrix_set (m1, 1, 1, 1); gsl_matrix_set (m1, 1, 2, 2);
    //gsl_matrix_set (m1, 2, 0, 0); gsl_matrix_set (m1, 2, 1, 1); gsl_matrix_set (m1, 2, 2, 2);
    //gsl_matrix_set (m1, 3, 0, 0); gsl_matrix_set (m1, 3, 1, 2); gsl_matrix_set (m1, 3, 2, 2);

    //gsl_matrix_set_identity(m2);

    //gsl_matrix_set (m2, 0, 0, 1); gsl_matrix_set (m2, 0, 1, 1); gsl_matrix_set (m2, 0, 2, 0);// gsl_matrix_set (m2, 0, 3, 1);
    //gsl_matrix_set (m2, 1, 0, 0); gsl_matrix_set (m2, 1, 1, 1); gsl_matrix_set (m2, 1, 2, 0);// gsl_matrix_set (m2, 1, 3, 0);
    //gsl_matrix_set (m2, 2, 0, 1); gsl_matrix_set (m2, 2, 1, 0); gsl_matrix_set (m2, 2, 2, 1);// gsl_matrix_set (m2, 2, 3, 1);
*/
