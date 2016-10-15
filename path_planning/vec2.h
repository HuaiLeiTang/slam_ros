#ifndef VEC2_H
#define VEC2_H

#include "node.h"

class Vec2 {
public:
    Vec2();
    Vec2(double x,double y);
    Vec2(Node a, Node b);
    Vec2(Node a);
    //Vec2(Obstacle o);
    Vec2 operator+(Vec2 other);
    Vec2 operator-(Vec2 other);
    Vec2 operator*(double scalar);
    double operator*(Vec2 other);
    double Lenght();
    Vec2 Norm();
    void Print();
    double x;
    double y;
};

double SubtendedAngle(Vec2 a, Vec2 b, Vec2 c);
double SubtendedCos(Vec2 a, Vec2 b, Vec2 c);
#endif // VEC2_H
