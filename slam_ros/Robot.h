#ifndef ROBOT_H_INCLUDED
#define ROBOT_H_INCLUDED

#include <iostream>
#include <vector>
#include <array>

#include "std_msgs/Float32MultiArray.h"

#include "simplifyPath.h"
#include "lineFitting.h"

#define LINESIZE 100
#define SLAMSIZE 203 // = LINESIZE*2+3
#define MAHALANOBIS 0.4
#define LINENOISE 0.03
#define ENCODERNOISE 0.024
#define SIMULATIONOFF true


class Robot
{
private:

    /// \brief stored lines part of the extended robot state vector
    double y[LINESIZE*2 + 3];              //LINESIZE: number of lines stored
    /// \brief number of lines saved in memory
    int savedLineCount;
    /// \brief old polar coordinate points in world reference frame found by the sensors
    std::vector<polar_point> oldPoints;
    /// \brief old polar coordinate lines in world reference frame found by the sensor
    std::vector<line> oldLines;
    /// \brief polar coordinate points in robot reference frame stored for 1 SLAM cycle
    std::vector<polar_point> points;
    /// \brief number of line matches found in one SLAM cycle
    int matchesNum;

    /// \brief forward kinematics matrix
    double forKin[6];
    /// \brief inverse kinematics matrix
    double invKin[6];
    /// \brief derives the forward kinamatics matrix from the parameters of the robot
    void forwardKin();
    /// \brief derives the inverse kinematics matrix from the parameters of the robot
    void inverseKin();
    /// \brief calculates velocity vector of the robot from the angular velocity of the 2 wheels (using forward kinematics
    void fiToXi(double*, double*);
    /// \brief calculates the angular velocitiy of the 2 wheels from the velocity vector of th robot (using inverse kinematics)
    void xiToFi(double*, double*);


public:
    /// \brief Robot coordinates
    double xPos;
    double yPos;
    double thetaPos;

    /// \brief line endpoints stored for one cycle
    std_msgs::Float32MultiArray lineIntervals;

    /// \brief Robot covariance matrix
    double P_t0[SLAMSIZE*SLAMSIZE];         //203*203 (100 lines)

    Robot(double x, double y, double theta);
    ~Robot();

    /// \brief issues move commands to the robot
    void walk(double r, double theta);
    /// \brief requests an IR sensor measurement
    void robot2World1(double dist, double *pos);
    void robot2World2(double *rot);
    void normalizeRadian(double& rad);
    bool getEllipse(float axii[2], float& angle);
    void localize(const std::vector<line> &lines, float* rot = NULL, const double* encoder = NULL);
    void measure();
    void readOdometry(double &left, double &right);
};


#endif // ROBOT_H_INCLUDED



/*
------------------ QUIZ2 -----------------
-----MOTION MODEL
poláris koordinátából, 3-os koordináta
f = @(x, u) [x(1) + u(1)*cos(x(3) + u(2)); x(2) + u(1)*sin(x(3) + u(2)); x(3) + u(2)];
Fx = @(x,u) [1 0 -u(1)*sin(x(3) + u(2)); 0 1 u(1)*cos(x(3) + u(2)); 0 0 1]
Fu = @(x,u) [cos(x(3) + u(2)) -u(1)*sin(x(3) + u(2)); sin(x(3) + u(2)) u(1)*cos(x(3) + u(2)); 0 1]

(x, u):u[x, y, theta]
f = [x(1) + u()

%%% SECTION FOR SELF-EVALUATION. PLEASE DO NOT EDIT BELOW THIS LINE %%%

x = [1.; 2. ; pi/4]
u = [.1; pi/8]

f_eval = f(x,u)
Fx_eval = Fx(x,u)
Fu_eval = Fu(x,u)


-----MEASUREMENT MODEL
h = @(x, m) [cos(x(3)) sin(x(3)); -sin(x(3)) cos(x(3))]*(m - x(1:2))
Hx = @(x,m) [-cos(x(3)) -sin(x(3)) (x(1)-m(1))*sin(x(3))+(m(2)-x(2))*cos(x(3)); sin(x(3)) -cos(x(3)) (x(1)-m(1))*cos(x(3))+(x(2)-m(2))*sin(x(3))];

%%% SECTION FOR SELF-EVALUATION. PLEASE DO NOT EDIT BELOW THIS LINE %%%

x = [1.; 0. ; pi/4]
m = [3.; 4.]

h_eval = h(x, m)
Hx_eval = Hx(x, m)

---------------- EKF LOCALIZATION ------------------
-----MOTION MODEL

f = @(x, u) x+u;
Fx = @(x,u) [1 0; 0 1];
Fu = @(x,u) [1 0; 0 1];

%%% SECTION FOR SELF-EVALUATION. PLEASE DO NOT EDIT BELOW THIS LINE %%%

x = [1;2];
u = [3;4];

f_eval = f(x,u)
Fx_eval = Fx(x,u)
Fu_eval = Fu(x,u)


----MEASUREMENT MODEL
h = @(m, x_prior) m - x_prior;
Hx = @(m, x_prior) [-1 0; 0 -1];

%% SECTION FOR SELF-EVALUATION. PLEASE DO NOT EDIT BELOW THIS LINE %%%

x_prior = [1;2]
m = [3;3]

h_eval = h(m,x_prior)
Hx_eval = Hx(m,x_prior)

------ UNCEARTAINITY PROPAGATION
P_prior = @(Fx,Fu,P,Q) Fx*P*Fx' + Fu*Q*Fu';

%% SECTION FOR SELF-EVALUATION. PLEASE DO NOT EDIT BELOW THIS LINE %%%

Fx = [2 1;1 2]
Fu = [2 1;1 2]
P = [1 0;0 1]
Q = [.5 0;0 .5]

P_prior_eval = P_prior(Fx,Fu,P,Q)

------ UPDATE STEP
y = @(z, h) z - h;
S = @(Hx, P_prior, R) Hx*P_prior*Hx' + R;
K = @(Hx, P_prior, S) P_prior*Hx*inv(S);
x_posterior = @(x_prior, K, y) x_prior + K*y;
P_posterior = @(Hx, P_prior, K) P_prior - P_prior*Hx*K';

%%% SECTION FOR SELF-EVALUATION. PLEASE DO NOT EDIT BELOW THIS LINE %%%

z = [1.1;1.9]
h = [1;2]
Hx = [2 1;1 2]
P_prior = [1 0;0 1]
R = [.5 0; 0 .5]
x_prior = [1;2]

y_eval = y(z,h)
S_eval = S(Hx, P_prior, R)
K_eval = K(Hx, P_prior, S_eval)
x_posterior_eval = x_posterior(x_prior, K_eval, y_eval)
P_posterior_eval = P_posterior(Hx, P_prior, K_eval)



----- SIMUALATING
 while systemRunning()

         u = queryMotionSensor();

         x_hat = f(x,u);
         Fx_hat = Fx(x,u);
         Fu_hat = Fu(x,u);
         P_hat = P_prior(Fx_hat,Fu_hat,P,Q);

         [z, m] = queryLandmarkSensor();

         if ~isempty(z)
                 z_hat = h(m, x_hat);
                 Hx_hat = Hx(m, x_hat);

                 y_hat = y(z, z_hat);
                 S_hat = S(Hx_hat, P_hat, R);
                 K_hat = K(Hx_hat, P_hat, S_hat);

                 x = x_posterior(x_hat, K_hat, y_hat);
                 P = P_posterior(Hx_hat, P_hat, K_hat);
         else
                 x = x_hat;
                 P = P_hat;
         end
 end









*/
