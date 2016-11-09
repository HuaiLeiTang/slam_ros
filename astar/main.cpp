////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// STL A* Search implementation
// (C)2001 Justin Heyes-Jones
//
// Finding a path on a simple grid maze
// This shows how to do shortest path finding using A*

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "stlastar.h" // See header for copyright and usage information

#include <iostream>
#include <stdio.h>
#include <math.h>
#include "gridmap.h"
#include"ros/ros.h"
#include "vec2.h"




using namespace std;
GridMap* gmap;




// Main

int main( int argc, char *argv[] )
{

    cout << "STL A* Search implementation\n(C)2001 Justin Heyes-Jones\n";
    ros::init(argc, argv, "start");
    ros::NodeHandle n;
    ros::Rate r(10);
    GridMap tempmap(37,200,&n);
    vector<AncientObstacle*> obs;
    AncientObstacle* newobs = new StraightObstacle(Vec2(50*37,0),Vec2(0,50*37));
    obs.push_back(newobs);
    newobs = new StraightObstacle(Vec2(-50*37,0),Vec2(0,-50*37));
    obs.push_back(newobs);
    newobs = new StraightObstacle(Vec2(-50*37,0),Vec2(0,50*37));
    obs.push_back(newobs);
    newobs = new StraightObstacle(Vec2(50*37,0),Vec2(0,-50*37));
    obs.push_back(newobs);
  /*  newobs = new StraightObstacle(Vec2(15*37,100*37),Vec2(100*37,20*37));
    obs.push_back(newobs);*/
    gmap = &tempmap;
    tempmap.DrawObstacle(obs);
    Astar pathfinder;
    pathfinder.FindPath(Vec2(250*37,250*37),Vec2(150*37,150*37));
    cout<<"oath end"<<endl;
    // Our sample problem defines the world as a 2d array representing a terrain
    // Each element contains an integer from 0 to 5 which indicates the cost
    // of travel across the terrain. Zero means the least possible difficulty
    // in travelling (think ice rink if you can skate) whilst 5 represents the
    // most difficult. 9 indicates that we cannot pass.

    // Create an instance of the search class...

  /*  AStarSearch<MapSearchNode> astarsearch;

    unsigned int SearchCount = 0;

    const unsigned int NumSearches = 1;
    vector<Vec2> vecpath;
    vecpath.reserve(10000);
    while(SearchCount < NumSearches)
    {


    }

    PotencialDistort(vecpath);
    PotencialDistort(vecpath);
    PotencialDistort(vecpath);
    PotencialDistort(vecpath);
    for(int i = 0; i < vecpath.size();i++) {
        gmap->data[gmap->mapHeight*(int)vecpath[i].y + (int)vecpath[i].x] = KNOWN;
    }
    cout<<"OCCUPACÍ "<<gmap->gridObstacles.size()<<endl;*/
    gmap->PublishMap();
    cout<<"publis"<<endl;
    r.sleep();
    ros::spinOnce();
    return 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
