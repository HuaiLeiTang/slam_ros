#include "lineFitting.h"
#include <fstream>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>

using namespace std;

line::line()
{
    alfa = 0;
    r = 0;
    C_AR = NULL;
}

line::line(double alfa, double r)
{
    this->alfa = alfa*(PI/180);
    this->r = r;
    b = (this->r)/sin(this->alfa);
    m = - 1 / tan(this->alfa);
}

double line::value_y(double x)
{
        return b + x*m;
}


vector<Point> line::line_graf(double x_1,double x_2)
{
    vector<Point> temp;
    Point temp_point;
    double x_i = x_1;
    double res = 1; // resolusion
    while( x_i < x_2 )
    {
        temp_point.x = x_i;
        temp_point.y = this->value_y(x_i);
        temp.push_back(temp_point);
        x_i = x_i + res;
    }
    return temp;
}

void line::WriteCov(void) {
    double *d = C_AR->data;
    printf("[ %g,%g\n",d[0],d[1]);
    printf(" %g,%g]\n",d[2],d[3]);
    cout<<"Determ: "<<d[0]*d[3] - d[1]*d[2]<<endl;
    cout<<endl;
}

/*__________Polar Point_______________________________*/
polar_point::polar_point()
{
    alfa = 0;
    r = 0;
    weight = 1;
    variance = 1;
    in_line = true;
}

polar_point::polar_point(double alfa, double r)
{
    this->alfa = alfa*(PI/180);
    this->r = r;
    weight = 1;
    in_line = true;
}

polar_point::polar_point(double alfa, double r, double weight)
{
    this->alfa = alfa*(PI/180);
    this->r = r;
    this->weight = weight;
    in_line = true;
}

bool operator==(const polar_point& this_one, const polar_point& other_one) // find algorithm require operator==
{
    if( (this_one.alfa == other_one.alfa) && (this_one.r == other_one.r) )
        return true;
    else
        return false;
}

void residual_error(vector<polar_point> pol_points,double& sum,line split_line)
{
    if(pol_points.size() < 2) {
        sum = 0;
    }
    else
    {
        vector<Point> temp_points;
        temp_points = polar2descart(pol_points);

        Point firstpoint;
        Point lastpoint;
        double dist;
        double sum_dist;

        firstpoint.x = temp_points[0].x;
        firstpoint.y = split_line.value_y(temp_points[0].x);

        lastpoint.x = temp_points[temp_points.size()-1].x;
        lastpoint.y = split_line.value_y(temp_points[temp_points.size()-1].x);
        Point p=lastpoint-firstpoint;

        for(int i=1;i < temp_points.size();i++)
        {
            Point pp=temp_points[i]-firstpoint;
            dist=fabs(pp * p) / p.Norm();
            sum_dist = sum_dist + dist;
        }

        sum  = sum_dist;
    }
}

std::vector<polar_point> descart2polar(std::vector<Point>& Points)
{
    std::vector<polar_point> temp_vect;
    polar_point temp_point;
    for(int i = 0 ; i < Points.size();i++)
    {
        temp_point.r = sqrt( Points[i].x*Points[i].x + Points[i].y*Points[i].y );
        temp_point.alfa = atan2(Points[i].y,Points[i].x);
        temp_vect.push_back(temp_point);
    }
    return temp_vect;
}

std::vector<Point> polar2descart(std::vector<polar_point>& polar_Points)
{
    std:vector<Point> temp_vect;
    Point temp_point;
    for(int i = 0;i < polar_Points.size(); i++)
    {
        temp_point.x = cos(polar_Points[i].alfa)*polar_Points[i].r;
        temp_point.y = sin(polar_Points[i].alfa)*polar_Points[i].r;
        temp_vect.push_back(temp_point);
    }
    return temp_vect;
}

line lineFitting(std::vector<Point>& Points)
{
    double r;
    double alfa;
    vector<polar_point> polar_data;
    double sum_1 = 0;
    double sum_2 = 0;
    double sum_3 = 0;
    double sum_4 = 0;
    double sum_r = 0;
    polar_data = descart2polar(Points);
    double sum_var;
    for(int i = 0; i < polar_data.size();i++)
        sum_var = sum_var + polar_data[i].weight;
    cout<<(double)Points.size()<<endl;
    cout<<sum_var<<endl;


    for(int i= 0; i < polar_data.size();i++)
        for(int j = i + 1; j < polar_data.size(); j++)
            sum_1 = sum_1 + polar_data[i].weight*polar_data[j].weight*polar_data[i].r*polar_data[j].r*sin(polar_data[i].alfa + polar_data[j].alfa);

    for(int i = 0;i < polar_data.size();i++)
        sum_2 = sum_2 + ( polar_data[i].weight - sum_var)*polar_data[i].r*polar_data[i].r*sin(2*polar_data[i].alfa);


    for(int i= 0; i < polar_data.size();i++)
        for(int j = i + 1; j < polar_data.size(); j++)
            sum_3 = sum_3 + polar_data[i].weight*polar_data[j].weight*polar_data[i].r*polar_data[j].r*cos(polar_data[i].alfa + polar_data[j].alfa);

    for(int i = 0;i < polar_data.size();i++)
        sum_4 = sum_4 + (polar_data[i].weight - sum_var)*polar_data[i].r*polar_data[i].r*cos(2*polar_data[i].alfa);

    double wi = (double)polar_data.size();

    alfa = 0.5*atan2( (2/wi)*sum_1 + (1/wi)*sum_2, (2/wi)*sum_3 +(1/wi)*sum_4 );

    for(int i = 0;i < polar_data.size();i++)
        sum_r = sum_r + polar_data[i].weight*polar_data[i].r*cos(polar_data[i].alfa - alfa);

    line linefit(alfa*180/PI,sum_r/sum_var);
    return linefit;
}


line lineFitting(std::vector<polar_point>& polar_data)
{

    double r;
    double alfa;

    double sum_1 = 0;
    double sum_2 = 0;
    double sum_3 = 0;
    double sum_4 = 0;
    double sum_r = 0;
    double wi = 0;
    for(int i = 0;i < polar_data.size();i++)
    {
        wi = wi + polar_data[i].weight;
    }
    for(int i= 0; i < polar_data.size();i++)
        for(int j = i + 1; j < polar_data.size(); j++)
            sum_1 = sum_1 + polar_data[i].weight*polar_data[j].weight*polar_data[i].r*polar_data[j].r*sin(polar_data[i].alfa +  polar_data[j].alfa);

    for(int i = 0;i < polar_data.size();i++)
    {
        sum_2 = sum_2 + (polar_data[i].weight - wi)*polar_data[i].weight*polar_data[i].r*polar_data[i].r*sin(2*polar_data[i].alfa);
    }


    for(int i= 0; i < polar_data.size();i++)
        for(int j = i + 1; j < polar_data.size(); j++)
            sum_3 = sum_3 + polar_data[i].weight*polar_data[j].weight*polar_data[i].r*polar_data[j].r*cos(polar_data[i].alfa + polar_data[j].alfa);

    for(int i = 0;i < polar_data.size();i++)
    {
        sum_4 = sum_4 + (polar_data[i].weight - wi)*polar_data[i].weight*polar_data[i].r*polar_data[i].r*cos(2*polar_data[i].alfa);
    }

    alfa = 0.5*atan2( (2/wi)*sum_1 + (1/wi)*sum_2, (2/wi)*sum_3 +(1/wi)*sum_4 );

    for(int i = 0;i < polar_data.size();i++)
        sum_r = sum_r + polar_data[i].weight*polar_data[i].r*cos(polar_data[i].alfa - alfa);

    line linefit(alfa*180/PI,sum_r/wi);
    return linefit;
}


FuncParam::FuncParam(vector<polar_point> initData,int iPut,int oPut) {
	covData = initData;
	inPut = iPut;
	outPut = oPut;
}

double Func(double x, void * params)
{
	FuncParam* param = (FuncParam*)(params);
	int inPut = param->inPut;
	int outPut = param->outPut;
	if(inPut > param->covData.size() - 1) {
		param->covData[inPut - param->covData.size()].alfa = x;
	}
	else {
		param->covData[inPut].r = x;
	}
	line diff = lineFitting(param->covData);
	if(outPut == 0) {
		return diff.r;
	}
	else {
		return diff.alfa;
	}

}
/*
gsl_matrix* Covariancia(vector<polar_point> cov_data) {
    gsl_matrix *F_pq = gsl_matrix_alloc(2,(size_t)2*cov_data.size()); // 2*n size Row a
    gsl_matrix *C_x = gsl_matrix_alloc((size_t)2*cov_data.size(),(size_t)2*cov_data.size());
    gsl_matrix *F_pq_t = gsl_matrix_alloc((size_t)2*cov_data.size(),2);
    gsl_matrix *C_ar = gsl_matrix_alloc(2,2);
    gsl_matrix *F_pg_C_x = gsl_matrix_alloc(2,(size_t)2*cov_data.size());
    for(int i = 0; i < cov_data.size();i++) {
        gsl_matrix_set(C_x,i,i,cov_data[i].variance*cov_data[i].variance);
    }
    for(int i = 0; i < cov_data.size(); i++) {
    	FuncParam params(cov_data,i,1);
    	gsl_function F;
    	double result, abserr;
    	F.function = &Func;
    	F.params = &params;
        gsl_deriv_forward(&F, cov_data[i].r, 1e-8, &result, &abserr);
    	gsl_matrix_set(F_pq,0,i,result);
    	params.covData = cov_data;
    	params.outPut = 0;
    	gsl_deriv_forward(&F, cov_data[i].r, 1e-8, &result, &abserr);
    	gsl_matrix_set(F_pq,1,i,result);

    }
    for(int i = 0; i < cov_data.size(); i++) {
    	FuncParam params(cov_data,i + cov_data.size(),1);
    	gsl_function F;
    	double result, abserr;
    	F.function = &Func;
    	F.params = &params;
        gsl_deriv_forward(&F, cov_data[i].alfa, 1e-8, &result, &abserr);
    	gsl_matrix_set(F_pq,0,i + cov_data.size(),result);
    	params.covData = cov_data;
    	params.outPut = 0;
        gsl_deriv_forward(&F, cov_data[i].alfa, 1e-8, &result, &abserr);
    	gsl_matrix_set(F_pq,1,i + cov_data.size(),result);
    }
    gsl_matrix_transpose_memcpy(F_pq_t,F_pq);

    gsl_blas_dgemm(CblasNoTrans,CblasNoTrans,1.0,F_pq,C_x,0.0,F_pg_C_x);//F_pq*C_x
    gsl_blas_dgemm(CblasNoTrans,CblasNoTrans,1.0,F_pg_C_x,F_pq_t,0.0,C_ar);
    ofstream matrix;
    return C_ar;
}*/

gsl_matrix* Covariancia(vector<polar_point> cov_data)
{
    gsl_matrix *F_pq = gsl_matrix_alloc(2,(size_t)2*cov_data.size()); // 2*n size Row a
    gsl_matrix *C_x = gsl_matrix_alloc((size_t)2*cov_data.size(),(size_t)2*cov_data.size());
    gsl_matrix *F_pq_t = gsl_matrix_alloc((size_t)2*cov_data.size(),2);
    gsl_matrix *C_ar = gsl_matrix_alloc(2,2);
    gsl_matrix *F_pg_C_x = gsl_matrix_alloc(2,(size_t)2*cov_data.size());
    double repo;
    line alfa_r_Pi_Qi = lineFitting(cov_data);
    line alfa_r_Pi_Qi_eps;
    double eps = 0.1;
    double alfa_derivat;
    double r_derivat;
    for(int i = 0;i < cov_data.size();i++)
    {
        repo = cov_data[i].r;
        cov_data[i].r = cov_data[i].r + eps;
        alfa_r_Pi_Qi_eps = lineFitting(cov_data);
        alfa_derivat = (alfa_r_Pi_Qi_eps.alfa - alfa_r_Pi_Qi.alfa)/eps;
        r_derivat = (alfa_r_Pi_Qi_eps.r - alfa_r_Pi_Qi.r)/eps;
        gsl_matrix_set(F_pq,0,i,alfa_derivat);
        gsl_matrix_set(F_pq,1,i,r_derivat);
        cov_data[i].r = repo;
    }


    for(int i = 0; i < cov_data.size();i++)
    {
        gsl_matrix_set(C_x,i,i,cov_data[i].variance*cov_data[i].variance);
    }

    int j = 0;
    for(int i = cov_data.size(); i < 2*cov_data.size();i++)
    {
        repo = cov_data[j].alfa;
        cov_data[j].alfa = cov_data[j].alfa + eps;
        alfa_r_Pi_Qi_eps = lineFitting(cov_data);
        alfa_derivat = (alfa_r_Pi_Qi_eps.alfa - alfa_r_Pi_Qi.alfa)/eps;
        r_derivat = (alfa_r_Pi_Qi_eps.r - alfa_r_Pi_Qi.r)/eps;
        gsl_matrix_set(F_pq,0,i,alfa_derivat);
        gsl_matrix_set(F_pq,1,i,r_derivat);
        cov_data[j].alfa = repo;
        j++;
    }

    gsl_matrix_transpose_memcpy(F_pq_t,F_pq);

    gsl_blas_dgemm(CblasNoTrans,CblasNoTrans,1.0,F_pq,C_x,0.0,F_pg_C_x);//F_pq*C_x
    gsl_blas_dgemm(CblasNoTrans,CblasNoTrans,1.0,F_pg_C_x,F_pq_t,0.0,C_ar);
    ofstream matrix;
    return C_ar;
}
/*_____________Line Extracion_______________*/


lineXtracion::lineXtracion()
{

}

lineXtracion::lineXtracion(vector<polar_point>& pol_points)
{
    this->pol_data = pol_points;
}

lineXtracion::lineXtracion(vector<Point>& points)
{
    this->pol_data = descart2polar(points);
}

void lineXtracion::Filter(int window)
{
    for(int i = 0; i < pol_data.size();i++)
        pol_data[i].r = (pol_data[i].r + pol_data[i - 1].r)*0.5;
}

void lineXtracion::Extract(void)
{
    simplifyPath black_magic;
    vector<pair<line,vector<polar_point> > > temp_result = black_magic.simplifyWithRDP(this->pol_data);
    for(int i = 0; i < temp_result.size(); i++)
    {
        Fitlines.push_back(temp_result[i].first);
        fit_pol_points.push_back(temp_result[i].second[0]);
        fit_pol_points.push_back(temp_result[i].second[1]);
    }
}

vector<Point> lineXtracion::Result_export(void)
{
    this->fit_points = polar2descart(this->fit_pol_points);
    vector<Point> temp = polar2descart(this->pol_data);
    vector<Point> point_result;
    temp.clear();
    int j = 0;
    for(int i = 0; i < this->Fitlines.size();i++)
        {
            temp = this->Fitlines[i].line_graf(fit_points[j].x,fit_points[j + 1].x);
            point_result.insert(point_result.end(),temp.begin(),temp.end());
            temp.clear();
            j = j + 2;
        }
    return point_result;
}

void lineXtracion::Export_polar()
{
    ofstream output_file;
    output_file.open("p_data.txt");
    double t_alfa;
    for(int i = 0; i < fit_pol_points.size();i++)
    {
        if( fit_pol_points[i].alfa < 0)
            t_alfa = 2*PI + fit_pol_points[i].alfa;
        else
            t_alfa = fit_pol_points[i].alfa;
        output_file<<fit_pol_points[i].r<<","<<t_alfa<<endl;
    }
}

void lineXtracion::Export_polar_data(void)
{
    ofstream output_file;
    output_file.open("p_raw_data.txt");
    double t_alfa;
    for(int i = 0; i < this->pol_data.size();i++)
    {
        if( pol_data[i].alfa < 0)
            t_alfa = 2*PI + pol_data[i].alfa;
        else
            t_alfa = pol_data[i].alfa;
        output_file<<pol_data[i].r<<","<<t_alfa<<endl;
    }
}

vector<line> LineExtraction(vector<polar_point>& pol_points) {
    int len = pol_points.size() - 1;
    double dist[len];
    for(int i = 0; i < len;i++) {
        dist[i] = pow(pow(pol_points[i].r,2) + pow(pol_points[i + 1].r,2) - 2*pol_points[i].r*pol_points[i + 1].r*cos(pol_points[i + 1].alfa - pol_points[i].alfa),2);
    }
    double sum_di;
    for(int i = 0; i < len; i++) {
        sum_di = sum_di + dist[i];
    }
    double avr = sum_di/len;
    vector<int> split;
    for(int i = 0; i < len; i++) {
        if(dist[i] > avr ) {
            split.push_back(i + 1);
        }
    }
    int segmens_num = split.size() + 1;
    vector<polar_point> segmens[segmens_num];
    vector<polar_point>::iterator iter = pol_points.begin();
    for(int j = 0; j < split.size() - 1; j++) {
        segmens[j + 1].insert(segmens[j + 1].begin(),iter + split[j],iter + split[j + 1]);
    }
    segmens[0].insert(segmens[0].begin(),iter,iter + split[0]);
    segmens[split.size()].insert(segmens[split.size()].begin(),iter + split[split.size() - 1],pol_points.end());
    vector<line> lines;
    lineXtracion get_lines[segmens_num];
    for(int i = 0; i < segmens_num; i++) {
        get_lines[i] = lineXtracion(segmens[i]);
    }
    for(int i = 0; i < segmens_num; i++) {
        get_lines[i].Extract();
        lines.insert(lines.end(),get_lines[i].Fitlines.begin(),get_lines[i].Fitlines.end());
    }
    return lines;
}
