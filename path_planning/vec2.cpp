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
    return sqrt(pow(x,2) + pow(y,2));
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
