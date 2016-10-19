#ifndef VEC2_H
#define VEC2_H

#include "node.h"
#include <iostream>

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
    void Merolegese();
    void RoundPoz90();
    void RoundMinusz90();
    double x;
    double y;
};

double Distance(Vec2 a, Vec2 b);
double SubtendedAngle(Vec2 a, Vec2 b, Vec2 c);
double SubtendedCos(Vec2 a, Vec2 b, Vec2 c);
double SkalarCos(Vec2 a, Vec2 b);
std::ostream& operator<<(std::ostream& os,const Vec2 a);
inline bool operator==(const Vec2& a, const Vec2& b) {
    if ( (a.x == b.x) && (a.y == b.y) ) {
        return true;
    }
    else {
        return false;
    }
}

inline bool operator!=(const Vec2& a, const Vec2& b){
    return !(a == b);
}
#endif // VEC2_H
