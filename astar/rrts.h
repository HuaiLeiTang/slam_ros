#ifndef RRTS_H
#define RRTS_H

#include "node.h"
#include "vec2.h"
#include "simplifyPath.h"
#include <utility>
#include <iostream>
#include <algorithm>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <fstream>

enum obstacleType { pointObstacle, lineObstacle };

class AncientObstacle {
public:
    virtual bool IsCollision(Node inGraf, Node* randNode, bool edit = true) = 0;
    virtual void Draw(std::ofstream& out) = 0;
    virtual Vec2 FirstPoint() = 0;
    virtual Vec2 EndPoint() = 0;
    virtual Vec2 FirstUp() = 0;
    virtual Vec2 EndUp() = 0;
    virtual Vec2 FirstDown() = 0;
    virtual Vec2 EndDown() = 0;
    int type;
};

class Obstacle: public AncientObstacle {
public:
    Obstacle();
    Obstacle(double x, double y);
    bool IsCollision(Node inGraf, Node* randNode, bool edit = true);
    void Draw(std::ofstream& out);
    void CircVecDist(Node* random, Node graf);
    double x;
    double y;
    Vec2 FirstPoint();
    Vec2 EndPoint();
    static constexpr double r = 30;
};

class StraightObstacle: public AncientObstacle {
public:
    StraightObstacle();
    StraightObstacle(Vec2 firstPoint, Vec2 endpoint);
    StraightObstacle(polar_point firstPoint, polar_point endPoint);
    bool IsCollision(Node inGraf, Node* randNode, bool edit = true);
    void Draw(std::ofstream& out);
    bool Between(Vec2 m);
    Vec2 FirstPoint();
    Vec2 EndPoint();
    Vec2 FirstUp();
    Vec2 EndUp();
    Vec2 FirstDown();
    Vec2 EndDown();
    Vec2 firstPoint;
    Vec2 endPoint;
    Vec2 firstUp;
    Vec2 firstDown;
    Vec2 endUp;
    Vec2 endDown;
    static constexpr double r = 25;
};

class RRTs {
public:
    RRTs(std::vector<AncientObstacle*> map,Node firstNode,double maxX, double maxY);
    RRTs(double maxX, double maxY);
    Node* ClosestNode(Node* randNode);
    std::vector<AncientObstacle*> obstacles;
    std::vector<Node*> graf;
    Node* RandNode(void);
    bool IsPartOfGraf(Node* newNode);
    void InsertToGraf(Node * newNode);
    void PathPlaning(Node goal);
    void ExportGraf();
    double maxMapSizeX;
    double maxMapSizeY;
    Node goal;
    std::vector<Node*> path;
    std::vector<Node> reducedPath;
    std::vector<Vec2> sendPath;
    std::vector<Node> dijkPath;
    std::vector<Vec2> visibleGraft;
    void SetPose(Vec2 pose);
    void Reset();
    void AddObstacles(std::vector<AncientObstacle*> newobs);
};

#endif // RRTS_H
