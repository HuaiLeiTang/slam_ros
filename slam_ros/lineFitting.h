#ifndef __lineFitting__H__
#define __lineFitting__H__

#include "simplifyPath.h"

#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_deriv.h>

#define PI 3.14159265

using namespace std;


std::vector<polar_point> descart2polar(std::vector<Point>& Points);

std::vector<Point> polar2descart(std::vector<polar_point>& Points);
Point polar2descart(polar_point polar_Points);
polar_point descart2polar(Point Points);

void polar2descart(std::vector<polar_point>& polar_Points, std::vector<Point>& points);

gsl_matrix* Covariancia(vector<polar_point> cov_data);

line lineFitting(std::vector<Point>& Points);
line lineFitting(std::vector<polar_point>& polar_data);

class FuncParam {
public:
	FuncParam(vector<polar_point> initData,int iPut,int oPut);
	vector<polar_point> covData;
	int outPut;
	int inPut;
};

double Func(double x, void* params);

class lineXtracion
{
    public:
        vector<polar_point> fit_pol_points;
        vector<Point> fit_points;
        lineXtracion();
        lineXtracion(vector<polar_point>& pol_points);
        lineXtracion(vector<Point>& points);
        void Extract(void);//Extract line from pol_data
        void Filter(int window); // Simple moving avarage "filter"
        void Export_polar();
        void Export_polar_data();
        vector<Point> Result_export(void);
        vector<line> Fitlines;
        vector<polar_point> pol_data;
};

void residual_error(vector<polar_point> pol_points,double& sum,line split_line);
vector<line> LineExtraction(vector<polar_point>& pol_points);
#endif // __lineFitting__H__

