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
#include "geometry_msgs/Transform.h"
#include "Robot.h"
#include "Vrep.h"


int main(int argc,char* argv[])
{
    
	int argc2 = 0;
	char** argv2 = NULL;
	ros::init(argc2, argv2, "slam");

    Robot* rover = new Robot(0, 0, 0);
    /*Vrep* vrep = new Vrep(argc, argv);

    if(vrep->error != NO_ERROR)
    {
        return 1;
    }
    vrep->loop(rover);
    delete vrep;*/

    std::vector<line> lines;
    lines.reserve(10);
    lines.push_back(line(0.0, 4.0));
    lines.push_back(line(M_PI/M_PI*182.0, 6.2));
    double rot[2] = {10*M_PI, 10*M_PI};
    rover->localize(rot, lines);

    ros::NodeHandle nh;
    ros::Publisher pubCov = nh.advertise<geometry_msgs::Transform>("slam_ros/robotCov", 100);
	while(ros::ok()){
	
	int c = getchar();
	if(c == 119){
		std::cout << "FORWARD, MARCH!";
	}
	ros::spinOnce();
	usleep(10000);
    geometry_msgs::Transform msg;
    msg.translation.x = rover->xPos;
    msg.translation.y = rover->yPos;
    msg.translation.z = rover->thetaPos;
    pubCov.publish(msg);
	}
	
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
