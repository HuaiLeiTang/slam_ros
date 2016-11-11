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
#include "stlastar.h"

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
    std::vector<int> gridObstacles;
    void DrawPerfectObs(std::vector<AncientObstacle*> obstacles);
    std::vector<std::vector<int>> targets;
    ros::Publisher mapPublisher;
    ros::NodeHandle* handle;
    void SetGrid(Vec2 grid, int value);
    int GetGridValue(float x, float y);
    void SetRobotPose(Vec2 pose);
    void DrawGridLine(Vec2 start, Vec2 end);
    geometry_msgs::Vector3 pose;
    void PublishMap(void);
    void DrawObstacle(std::vector<AncientObstacle*> obstacles);
    void DrawObstacle(Vec2 start, Vec2 end);
    std::vector<int> DrawLine(Vec2 firstPoint, Vec2 endPoint, int value, bool stoppable);
    std::vector<int*> DrawArc(double radius,Vec2 firstPoint, Vec2 endPoint, int value, bool stoppable);
    void DrawCircle();
    void MapUpdate(std::vector<line> newLines, geometry_msgs::Vector3 robotPose);
    Vec2 Vec2Quantization(Vec2 q);
    Vec2 Vec2QBaseVector(Vec2 qb);
    int MapIndex(Vec2 index);
    std::vector<int> DrawCircle(Vec2 start, double radius,double rad, int value, bool stoppable);
    std::vector<int> DrawNegativCircle(Vec2 start, double radius, double rad, int value, bool stoppable);
    void UpgradeKnownGrid(std::vector<AncientObstacle *> &obstacles);
    void UpgradeTargets(std::vector<AncientObstacle *> &obstaclesV);
    std::pair<Vec2,Vec2> ClosestFirst(AncientObstacle * obstacle);
    Vec2 MapIndexInverse(int index);
    int MapIndexNoOffset(Vec2 qindex);
    Vec2 QuantMapIndexInverse(int index);
    Vec2 NextGoal();
    std::vector<AncientObstacle*> SortObstacles(std::vector<AncientObstacle *> &obstacles);
    //----------A Star------------
    static constexpr double r = 35;
    static constexpr double senser = 500;
};


extern GridMap* gmap; //ertéékeadááásás

class MapSearchNode
{
public:
    int x;	 // the (x,y) positions of the node
    int y;
    MapSearchNode() { x = y = 0; }
    MapSearchNode( int px, int py ) { x=px; y=py; }
    int GetMap( int x, int y );
    float GoalDistanceEstimate( MapSearchNode &nodeGoal );
    bool IsGoal( MapSearchNode &nodeGoal );
    bool GetSuccessors( AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node );
    float GetCost( MapSearchNode &successor );
    bool IsSameState( MapSearchNode &rhs );

    void PrintNodeInfo(vector<Vec2> &path);
    void PrintNodeInfo();
private:

};

class Astar {
public:
    Astar();
    bool FindPath(Vec2 start, Vec2 goal);
    void PathLines();
    Vec2 Gradient(Vec2 pose);
    void PotencialDistort(void);
    void SetGoalCharge();
    void Framing();
    void Reset();
    vector<Vec2> vecpath;
    vector<int> obsbuffer;
    vector<int> tempobs;
    Vec2 goalVec;
    Vec2 startVec;
    float goalCharge;

};




class ObsVec {
public:
    ObsVec();
    ObsVec(AncientObstacle* obs,Vec2 firstPoint);
    AncientObstacle* obspointer;
    Vec2 firstPoint;
};

#endif // GRIDMAP_H
