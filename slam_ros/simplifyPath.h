//
//  simplifyPath.h
//  RDP
//
//  Created by Yongjiao Yu on 14-6-13.
//  Copyright (c) 2014年 ___RCPD___. All rights reserved.
//


#ifndef __simplifyPath__H__
#define __simplifyPath__H__

#include <iostream>
#include <cstdlib>
#include <vector>
#include <cmath>
#include <algorithm>
#include <gsl/gsl_matrix.h>


//"Point" struct stand for each GPS coordinates(x,y). Methods related are used only for simplification of calculation in implementation.

using namespace std;

typedef struct Point{
    double x;
    double y;

    double operator*(Point rhs)const{
        return (this->x * rhs.y - rhs.x * this->y);
    }

    Point operator-(Point rhs)const{
        Point p;
        p.x=this->x-rhs.x;
        p.y=this->y-rhs.y;
        return p;
    }

    double Norm()const{
        return sqrt(this->x * this->x + this->y * this->y);
    }

}Point;


class polar_point
{
    public:
        polar_point();
        polar_point(double alfa, double r);
        polar_point(double alfa, double r, double weight);
        double alfa;
        double r;
        double weight;
        double variance;
        bool in_line;
};

class line
{
    public:
        line();
        line(double alfa, double r);
        double alfa;
        double r;
        double value_y(double x); // y = f(x) line discription
        void residual_error(vector<polar_point> pol_point,double& sum);
        gsl_matrix * C_AR; // covariance matrix of line parames alfa r
        vector<Point> line_graf(double x_1,double x_2); // general line points between x_1 ,x_2 interval
        void WriteCov(void);
    private:
        double b;
        double m;
};


class simplifyPath{
//"findMaximumDistance" used as part of implementation for RDP algorithm.
private:
    const std::pair<int, double> findMaximumDistance(const std::vector<Point>& Points)const;
    const std::pair<int, double> findMaximumDistance(const vector<Point>& Points,line& at_line)const;

//"simplifyWithRDP" returns the simplified path with a Point vector. The function takes in the paths to be simplified and a customerized thresholds for the simplication.
public:
    vector<pair<line,vector<polar_point> > > simplifyWithRDP(vector<polar_point>& polar_Points);
};

#endif


