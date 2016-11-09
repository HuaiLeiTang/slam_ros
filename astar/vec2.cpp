#include "vec2.h"
#include <cmath>
#include <iostream>

#define PI 3.141592

Vec2::Vec2():x(), y() {

}

Vec2::Vec2(double x, double y): x(x), y(y) {
}

Vec2::Vec2(Node a) {
    x = a.x;
    y = a.y;
}

Vec2::Vec2(Node a, Node b) {
    x = a.x - b.x;
    y = a.y - b.y;
}

Vec2 Vec2::operator +(Vec2 other) {
    Vec2 ret(x + other.x, y + other.y);
    return ret;
}

Vec2 Vec2::operator -(Vec2 other) {
    Vec2 ret(x - other.x, y - other.y);
    return ret;
}

Vec2 Vec2::operator *(double scalar) {
    Vec2 ret(x*scalar, y*scalar);
    return ret;
}

double Vec2::Lenght() {
    double temp = pow(x,2) + pow(y,2);
    if(temp > 0) {
        return sqrt(pow(x,2) + pow(y,2));
    }
    else {
        return 0;
    }
}

Vec2 Vec2::Norm() {
    double lenght = Lenght();
    Vec2 ret(x/lenght,y/lenght);
    return ret;
}

void Vec2::Print(void) {
    std::cout<<x<<" "<<y<<std::endl;
}

double Vec2::operator*(Vec2 other) {
    return this->x*other.x + this->y-other.y;
}

void Vec2::Merolegese() {
    double t = x;
    x = -y;
    y = t;
}

void Vec2::RoundMinusz90() {
    double t = x;
    x = y;
    y = t;
    y = -y;
}

void Vec2::RoundPoz90() {
    double t = x;
    x = y;
    y = t;
    x = -x;
}

void Vec2::Rotate(double theta) {
    x = cos(theta)*x - sin(theta)*y;
    y = x*sin(theta) + y*cos(theta);
}

double SubtendedAngle(Vec2 a, Vec2 b, Vec2 c) {
    double g = ((pow(a.Lenght(),2) + pow(b.Lenght(),2) - pow(c.Lenght(),2))/(2*a.Lenght()*b.Lenght()));
    if(g > 1) {
        return 0;
    }
    if(g < -1) {
        return PI;
    }

    return acos(g);
}

double SubtendedCos(Vec2 a, Vec2 b, Vec2 c) {
    double g = ((pow(a.Lenght(),2) + pow(b.Lenght(),2) - pow(c.Lenght(),2))/(2*a.Lenght()*b.Lenght()));
    if(g > 1) {
        return 1;
    }
    if(g < -1) {
        return -1;
    }
    return g;
}

double SkalarCos(Vec2 a, Vec2 b) {
    double skal = a.x*b.x + a.y*b.y;
    double r = skal/(a.Lenght()*b.Lenght());
    return r;
}

double Distance(Vec2 a, Vec2 b) {
    Vec2 t = a - b;
    return t.Lenght();
}

std::ostream& operator<<(std::ostream& os,const Vec2 a)
{
    os<<a.x<<" "<<a.y<<" ";
    return os;
}

