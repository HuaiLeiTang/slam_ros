#include "lineFitting.h"
#include <fstream>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <cmath>
#include <algorithm>

using namespace std;

line::line():lineInterval()
{
    alfa = 0;
    r = 0;
    C_AR = NULL;
}

line::line(double alfa, double r): lineInterval()
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
    temp_point.x = x_1;
    temp_point.y = this->value_y(x_1);
    temp.push_back(temp_point);
    temp_point.x = x_2;
    temp_point.y = this->value_y(x_2);
    temp.push_back(temp_point);
    return temp;
}

void line::SetEndPoints() {
    this->lineInterval[0].r = r/(cos(lineInterval[0].alfa - this->alfa));
    this->lineInterval[1].r = r/(cos(lineInterval[1].alfa - this->alfa));
    this->lineInterval[0].alfa = lineInterval[0].alfa + PI;
    this->lineInterval[1].alfa = lineInterval[1].alfa + PI;
    lineInterval[0].alfa = lineInterval[0].alfa > PI ? lineInterval[0].alfa-2.0*PI : lineInterval[0].alfa;
    lineInterval[1].alfa = lineInterval[1].alfa > PI ? lineInterval[1].alfa-2.0*PI : lineInterval[1].alfa;
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

polar_point::polar_point(double alfa, double r, double variance)
{
    this->alfa = alfa*(PI/180);
    this->r = r;
    this->weight = 1/pow(variance,2);
    this->variance = variance;
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

polar_point descart2polar(Point Points)
{
    polar_point temp_point;
    temp_point.r = sqrt( Points.x*Points.x + Points.y*Points.y );
    temp_point.alfa = atan2(Points.y,Points.x);
    return temp_point;
}

polar_point descart2polar(Vec2 Points) {
    polar_point temp_point;
    temp_point.r = sqrt( Points.x*Points.x + Points.y*Points.y );
    temp_point.alfa = atan2(Points.y,Points.x);
    return temp_point;
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

Point polar2descart(polar_point polar_Points)
{
    Point temp_point;
    temp_point.x = cos(polar_Points.alfa)*polar_Points.r;
    temp_point.y = sin(polar_Points.alfa)*polar_Points.r;
    return temp_point;
}


void polar2descart(std::vector<polar_point>& polar_Points, std::vector<Point>& points)
{
    Point temp_point;
    for(int i = 0;i < polar_Points.size(); i++)
    {
        temp_point.x = cos(polar_Points[i].alfa)*polar_Points[i].r;
        temp_point.y = sin(polar_Points[i].alfa)*polar_Points[i].r;
        points.push_back(temp_point);
    }
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

    alfa = 0.5*atan2((2/wi)*sum_1 + (1/wi)*sum_2, (2/wi)*sum_3 +(1/wi)*sum_4);

    for(int i = 0;i < polar_data.size();i++)
        sum_r = sum_r + polar_data[i].weight*polar_data[i].r*cos(polar_data[i].alfa - alfa);

    line linefit(alfa*180/PI,abs(sum_r/sum_var));
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

    alfa = 0.5*atan2( (2/wi)*sum_1 + (1/wi)*sum_2 , (2/wi)*sum_3 +(1/wi)*sum_4 );

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

gsl_matrix* Covariancia(vector<polar_point> cov_data) {
    gsl_matrix *F_pq = gsl_matrix_alloc(2,(size_t)2*cov_data.size()); // 2*n size Row a
    gsl_matrix *C_x = gsl_matrix_alloc((size_t)2*cov_data.size(),(size_t)2*cov_data.size());
    gsl_matrix *F_pq_t = gsl_matrix_alloc((size_t)2*cov_data.size(),2);
    gsl_matrix *C_ar = gsl_matrix_alloc(2,2);
    gsl_matrix *F_pg_C_x = gsl_matrix_alloc(2,(size_t)2*cov_data.size());
    for(int i = 0; i < cov_data.size();i++) {
        gsl_matrix_set(C_x,i,i,cov_data[i].variance*cov_data[i].variance); // TODO add alfa variance to C-x matrix
        gsl_matrix_set(C_x,i + cov_data.size(),i + cov_data.size(),cov_data[i].variance*cov_data[i].alfaVariance);
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
}

/*gsl_matrix* Covariancia(vector<polar_point> cov_data)
{
    gsl_matrix *F_pq = gsl_matrix_alloc(2,(size_t)2*cov_data.size()); // 2*n size Row a
    gsl_matrix *C_x = gsl_matrix_alloc((size_t)2*cov_data.size(),(size_t)2*cov_data.size());
    gsl_matrix *F_pq_t = gsl_matrix_alloc((size_t)2*cov_data.size(),2);
    gsl_matrix *C_ar = gsl_matrix_alloc(2,2);
    gsl_matrix *F_pg_C_x = gsl_matrix_alloc(2,(size_t)2*cov_data.size());
    double repo;
    line alfa_r_Pi_Qi = lineFitting(cov_data);
    if(alfa_r_Pi_Qi.alfa < 0) {
        alfa_r_Pi_Qi.alfa += PI;
    }
    line alfa_r_Pi_Qi_eps;
    double eps = 0.001;
    double alfa_derivat;
    double r_derivat;
    double alfa;
    for(int i = 0;i < cov_data.size();i++)
    {
        repo = cov_data[i].r;
        cov_data[i].r = cov_data[i].r + eps;
        alfa_r_Pi_Qi_eps = lineFitting(cov_data);
        if(alfa_r_Pi_Qi_eps.alfa  < 0) {
            alfa = alfa_r_Pi_Qi_eps.alfa + PI;
        }
        else {
            alfa = alfa_r_Pi_Qi_eps.alfa;
        }
        alfa_derivat = ( alfa - alfa_r_Pi_Qi.alfa)/eps;
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
        if(alfa_r_Pi_Qi_eps.alfa  < 0) {
            alfa = alfa_r_Pi_Qi_eps.alfa + PI;
        }
        else {
            alfa = alfa_r_Pi_Qi_eps.alfa;
        }
        alfa_derivat = (alfa - alfa_r_Pi_Qi.alfa)/eps;
        r_derivat = (alfa_r_Pi_Qi_eps.r - alfa_r_Pi_Qi.r)/eps;
        gsl_matrix_set(F_pq,0,i,alfa_derivat);
        gsl_matrix_set(F_pq,1,i,r_derivat);
        cov_data[j].alfa = repo;
        j++;
    }
    gsl_matrix_transpose_memcpy(F_pq_t,F_pq);

    gsl_blas_dgemm(CblasNoTrans,CblasNoTrans,1.0,F_pq,C_x,0.0,F_pg_C_x);//F_pq*C_x
    gsl_blas_dgemm(CblasNoTrans,CblasNoTrans,1.0,F_pg_C_x,F_pq_t,0.0,C_ar);
    return C_ar;
}*/
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
    for(int i = 0; i < this->Fitlines.size(); i++) {
        output_file<<Fitlines[i].lineInterval[0].r<<" "<<(Fitlines[i].lineInterval[0].alfa)<<endl;
        output_file<<Fitlines[i].lineInterval[1].r<<" "<<(Fitlines[i].lineInterval[1].alfa)<<endl;
    }
}

void lineXtracion::Export_polar_data(void)
{
    ofstream output_file;
    output_file.open("p_raw_data.txt");
    double t_alfa;
    for(int i = 0; i < this->pol_data.size();i++)
    {
        pol_data[i].alfa += PI;
        if( pol_data[i].alfa < 0)
            t_alfa = 2*PI + pol_data[i].alfa;
        else
            t_alfa = pol_data[i].alfa;
        output_file<<pol_data[i].r<<","<<t_alfa<<endl;
    }
}

int compareDouble (const void * a, const void * b)
{
  if ( *(double*)a <  *(double*)b ) return -1;
  if ( *(double*)a == *(double*)b ) return 0;
  if ( *(double*)a >  *(double*)b ) return 1;
}

void segmentation(vector<polar_point>& pol_points,vector<int>& split) {
    int len = pol_points.size() - 1;
    double dist[len];
    double mediandist[len];
    double max = 0;
    ofstream distfile;
    distfile.open("dist.txt");
    for(int i = 0; i < len;i++) {
        dist[i] = sqrt(pow(pol_points[i].r,2) + pow(pol_points[i + 1].r,2) - 2*pol_points[i].r*pol_points[i + 1].r*cos(pol_points[i + 1].alfa - pol_points[i].alfa));
        distfile<<dist[i]<<endl;
        if(dist[i] > max) {
            max = dist[i];
        }
    }
    for(int i = 0; i < len; i++) {
        mediandist[i] = dist[i];
    }
    qsort (mediandist,len,sizeof(double),compareDouble);
    double median;
    if(len % 2 == 0) {
        median = (mediandist[(len - 2)/2] + mediandist[(len - 2)/2 + 1])/2;
    }
    else {
        median = mediandist[(len - 1)/2];
    }
    for(int i = 0; i < len; i++) {
        if(mediandist[i] < median*5) {
            mediandist[i] = median*5;
        }

    }
    double sum_di;
    for(int i = 0; i < len; i++) {
        sum_di = sum_di + mediandist[i];
    }
    double avr = sum_di/len;
    distfile<<avr<<endl;
    double max_avr = max/avr;
    cout<<"max_avr: "<<max_avr<<endl;
    if(true) {
        for(int i = 0; i < len; i++) {
            if(dist[i] > 0.45) {
                split.push_back(i + 1);
            }
        }
    }
    cout<<"split num"<<split.size()<<endl;
}

void LineConversion(vector<line>& lines) {
   for(int i = 0; i < lines.size(); i++) {
        if(lines[i].r == 0) {
            delete lines[i].C_AR;
            lines.erase(lines.begin() + i);
            i--;
            continue;
        }
        if((lines[i].C_AR->data[0] < 0 ) || (lines[i].C_AR->data[3] < 0) ) {
            delete lines[i].C_AR;
            lines.erase(lines.begin() + i);
            i--;
            continue;
        }
        if(  isnan(lines[i].C_AR->data[0])|| isnan(lines[i].C_AR->data[1]) || isnan(lines[i].C_AR->data[2]) || isnan(lines[i].C_AR->data[3])) {
            delete lines[i].C_AR;
            lines.erase(lines.begin() + i);
            i--;
            continue;
        }
        if((lines[i].alfa) == 0 && (lines[i].r == 0)) {
            lines.erase(lines.begin() + i);
            i--;
            continue;
        }
        if((lines[i].C_AR->data[0] > 1) || (lines[i].C_AR->data[0] > 1)) {
            delete lines[i].C_AR;
            lines.erase(lines.begin() + i);
            i--;
            continue;
        }
    }
    for(int i = 0; i < lines.size(); i++) {
        if(lines[i].r < 0) {
            lines[i].r = abs(lines[i].r);
            if(lines[i].alfa < 0) {
                lines[i].alfa = PI + lines[i].alfa;
            }
            else {
                lines[i].alfa = -PI + lines[i].alfa;
            }
        }
    }
}

vector<line> LineExtraction(vector<polar_point>& pol_points) {
    vector<int> split;
    segmentation(pol_points,split);
    if(split.size() != 0) {
        rotate(pol_points.begin(),pol_points.begin() + split.back(),pol_points.end());
        split.clear();
        segmentation(pol_points,split);
        if(split.size() != 0) {
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
            lineXtracion plot;
            for(int i = 0; i < segmens_num; i++) {
                get_lines[i].Extract();
                lines.insert(lines.end(),get_lines[i].Fitlines.begin(),get_lines[i].Fitlines.end());
                plot.fit_pol_points.insert(plot.fit_pol_points.begin(),get_lines[i].fit_pol_points.begin(),get_lines[i].fit_pol_points.end());
                plot.pol_data.insert(plot.pol_data.begin(), get_lines[i].pol_data.begin(), get_lines[i].pol_data.end());
            }
            LineConversion(lines);
            plot.Fitlines = lines;
            plot.Export_polar();
            plot.Export_polar_data();
            return lines;
        }
        else {
            lineXtracion get_lines(pol_points);
            get_lines.Extract();
            LineConversion(get_lines.Fitlines);
            get_lines.Export_polar();
            get_lines.Export_polar_data();            
            return get_lines.Fitlines;
        }
    }
    else {
        lineXtracion get_lines(pol_points);
        get_lines.Extract();
        LineConversion(get_lines.Fitlines);
        get_lines.Export_polar();
        get_lines.Export_polar_data();
        return get_lines.Fitlines;
    }
}
