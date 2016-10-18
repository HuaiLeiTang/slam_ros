#ifndef GRIDMAP_H
#define GRIDMAP_H

#include "geometry_msgs/Vector3.h"
#include "ros/ros.h"
#include <stdlib.h>
#include "nav_msgs/OccupancyGrid.h"
#include <stdint.h>
#include "vec2.h"
#include "lineFitting.h"
#include "simplifyPath.h"
#include <math.h>
#include "rrts.h"

//#define mapIndex(sx, i, j) ((sx) * (j) + (i))
#define UNKNOWN -1
#define KNOWN 0
#define OCCUPANCY 100
#define TARGET 50


class GridMap {
public:
    GridMap(float resolution, int size,ros::NodeHandle* mapPublisher); // Map mérete (X,Y)  X,Y = 2*kvant * size; resolution m/cell 0.01
    float resolution;
    uint32_t mapWidth;
    uint32_t mapHeight;
    int dataSize;
    int size;
    Vec2 offset;
    int8_t* data;
    std::vector<std::vector<int*>> targets;
    ros::Publisher mapPublisher;
    ros::NodeHandle* handle;
    void SetGrid(Vec2 grid, int value);
    void SetRobotPose(geometry_msgs::Vector3 pose);
    geometry_msgs::Vector3 pose;
    void PublishMap(void);
    void DrawObstacle(std::vector<AncientObstacle*> obstacles);
    std::vector<int*> DrawLine(Vec2 firstPoint, Vec2 endPoint, int value, bool stoppable);
    std::vector<int*> DrawArc(double radius,Vec2 firstPoint, Vec2 endPoint, int value, bool stoppable);
    void DrawCircle();
    void MapUpdate(std::vector<line> newLines, geometry_msgs::Vector3 robotPose);
    Vec2 Vec2Quantization(Vec2 q);
    Vec2 Vec2QBaseVector(Vec2 qb);
    int MapIndex(Vec2 index);
    void DrawCircle(Vec2 start, double radius,double rad, int value, bool stoppable);
    void UpgradeKnownGrid(std::vector<AncientObstacle *> &obstacles);
    void UpgradeTargets(std::vector<AncientObstacle *> &obstacles);
    std::pair<Vec2,Vec2> ClosestFirst(AncientObstacle * obstacle);
    static constexpr double r = 10;
    static constexpr double senser = 300;
};

#endif // GRIDMAP_H
