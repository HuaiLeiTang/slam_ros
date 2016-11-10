#include "gridmap.h"

#define DEBUG_LISTS 0
#define DEBUG_LIST_LENGTHS_ONLY 0

ObsVec::ObsVec(AncientObstacle *obs, Vec2 firstPoint): firstPoint(firstPoint) {
    obspointer = obs;
}

inline bool operator==(const ObsVec& va, const ObsVec& vb) {
        return false;
}

bool operator<(const ObsVec& va, const ObsVec& vb)
{
    double alfa = atan2(va.firstPoint.y,va.firstPoint.x);
    alfa = alfa < 0 ? alfa + 2.0*M_PI : alfa;
    double beta = atan2(vb.firstPoint.y,vb.firstPoint.x);
    beta = beta < 0 ? beta + 2.0*M_PI : beta;
    if((alfa) < (beta))
        return true;
    else
        return false;
}

bool operator>(const ObsVec& va, const ObsVec& vb)
{
    double alfa = atan2(va.firstPoint.y,va.firstPoint.x);
    alfa = alfa < 0 ? alfa + 2.0*M_PI : alfa;
    double beta = atan2(vb.firstPoint.y,vb.firstPoint.x);
    beta = beta < 0 ? beta + 2.0*M_PI : beta;
    if((alfa) > (beta))
        return true;
    else
        return false;
}

GridMap::GridMap(float resolution, int size, ros::NodeHandle *handler): resolution(resolution), offset(size,size), mapHeight(2*size),
    mapWidth(2*size), targets(), size(size) {
    this->handle = handler;
    this->mapPublisher = handle->advertise<nav_msgs::OccupancyGrid>("/map",100);
    data = new int8_t[4*size*size];
    pose.x = 0;
    pose.y = 0;
    pose.z = 0;
    gridObstacles.reserve(size*size);
    dataSize = 4*size*size;
    for(int i = 0; i < dataSize; i++) {
            data[i] = UNKNOWN;
    }
}

void GridMap::PublishMap() {
    while(mapPublisher.getNumSubscribers() != 1) {

    }
    nav_msgs::OccupancyGrid msg;
    std_msgs::Header t_header;
    nav_msgs::MapMetaData t_into;
    t_into.height = mapHeight;
    t_into.width = mapWidth;
    t_into.resolution = resolution/100;
    msg.data.resize(dataSize);
    for(int i = 0; i < dataSize;i++) {
        msg.data[i] = data[i];
    }
    msg.info = t_into;
    mapPublisher.publish(msg);
    ros::spinOnce();
}

Vec2 GridMap::Vec2Quantization(Vec2 k) {
    Vec2 q;
    q.x = round(k.x/resolution - 0.5);
    q.y = round(k.y/resolution - 0.5);
    return q;
}

Vec2 GridMap::Vec2QBaseVector(Vec2 qb) {
    double fi = atan2(qb.y,qb.x);
    if((fi >= -22.5*PI/180) && (fi <= 22.5*PI/180)) {
        Vec2 a(1,0);
        return a;
    }
    if((fi >= 22.5*PI/180) && (fi <= 67.5*PI/180)) {
        Vec2 b(1,1);
        return b;
    }
    if( (fi >= 67.5*PI/180) && (fi <= 112.5*PI/180) ) {
        Vec2 c(0,1);
        return c;
    }
    if( (fi >= 112.5*PI/180) && (fi <= 157.5*PI/180) ) {
        Vec2 d(-1,1);
        return d;
    }
    if( (fi >= 157.5*PI/180) || (fi <= -157.5*PI/180) ) {
        Vec2 e(-1,0);
        return e;
    }
    if( (fi >= -157.5*PI/180 ) && (fi <= -112.5*PI/180) ) {
        Vec2 f(-1,-1);
        return f;
    }
    if( (fi >= -112.5*PI/180) && (fi <= -67.5*PI/180 ) ) {
        Vec2 g(0,-1);
        return g;
    }
    if( (fi >= -67.5*PI/180) && (fi <= -22.5*PI/180) ) {
        Vec2 h(1,-1);
        return h;
    }
}

int GridMap::MapIndex(Vec2 qindex) {
    Vec2 q_index = qindex + offset;
    return (int)(((int)mapWidth) *(int)(q_index.y) + (int)(q_index.x));
}

int GridMap::MapIndexNoOffset(Vec2 qindex) {
    Vec2 q_index = qindex;
    return (int)(((int)mapWidth) *(int)(q_index.y) + (int)(q_index.x));
}

void GridMap::SetGrid(Vec2 grid, int value) {
    if((MapIndex(grid) < 0 ) || (MapIndex(grid) > dataSize)) {
        cout<<"Invalid Vec2"<<endl;
        return;
    }
    if(value == OCCUPANCY) {
        data[MapIndex(grid)] = value;
        return;
    }
    if( (data[MapIndex(grid)] == KNOWN) || (data[MapIndex(grid)] == OCCUPANCY)) {
        return;
    }
    data[MapIndex(grid)] = value;
}

int GridMap::GetGridValue(float x, float y) {
    Vec2 grid(x,y);
    if((MapIndex(grid) < 0 ) || (MapIndex(grid) > dataSize)) {
        return OCCUPANCY;
    }
    else {
        return data[MapIndex(grid)];
    }
}

void GridMap::DrawGridLine(Vec2 start, Vec2 end) {
    if(((start.y*mapHeight + start.x) > dataSize) || (end.y*mapHeight+end.x > dataSize) ) {
        cout<<"Invalid Start or end Vector in DrawGridLine"<<endl;
        return;
    }
    Vec2 SE = end - start;
    SE = SE.Norm()*0.5;
    while(!(Distance(start,end) < 1)){
        data[(int)ceil(start.y)*mapHeight + (int)ceil(start.x)] = KNOWN;
        start = start + SE;
    }
}

std::vector<int> GridMap::DrawLine(Vec2 firstPoint, Vec2 endPoint, int value, bool stoppable) {
    Vec2 F = firstPoint;
    Vec2 kvantF;
    bool newtarget = true;
    bool newobstacle= true;
    Vec2 FE_norm = endPoint - firstPoint;
    FE_norm = FE_norm.Norm();
    FE_norm = FE_norm * (r/2);
    if(value == TARGET) {
        F = F + FE_norm*10;
    }
    vector<int> buf;
    while(true) {
        kvantF = Vec2Quantization(F);
        if(data[MapIndex(kvantF)] == TARGET && (value == TARGET)) {
            newtarget = false;
        }
        if(data[MapIndex(kvantF)] == OCCUPANCY && (value == OCCUPANCY)) {
            newobstacle = false;
        }
        SetGrid(kvantF,value);
        if((data[MapIndex(kvantF)] == TARGET) && newtarget && (value == TARGET)){
            buf.push_back(MapIndex(kvantF));
        }
        if((data[MapIndex(kvantF)] == OCCUPANCY) && newobstacle && (value == OCCUPANCY)){
            buf.push_back(MapIndex(kvantF));
        }
        newtarget = true;
        newobstacle = true;
        F = F + FE_norm;
        if(Distance(F,endPoint) < r/2) {
            break;
        }
        if(stoppable && (data[MapIndex(kvantF)] == OCCUPANCY)) {
            break;
        }
    }
    return buf;
}

void GridMap::DrawObstacle(std::vector<AncientObstacle *> obstacles) {
    Vec2 first;
    Vec2 end;
    vector<int> tempobs1;
    vector<int> tempobs2;
    vector<int> tempobs3;
    for(int i = 0; i < obstacles.size(); i++) {
        first = obstacles[i]->FirstUp();
        end = obstacles[i]->EndUp();
        tempobs1 = DrawLine(first,end,OCCUPANCY,false);
        first = obstacles[i]->FirstDown();
        end = obstacles[i]->EndDown();
        tempobs2 = DrawLine(first,end,OCCUPANCY,false);
        first = obstacles[i]->FirstPoint();
        end = obstacles[i]->EndPoint();
        Vec2 fe = first - end;
        fe = fe.Norm();
        Vec2 ef = fe*(-1);
        first = first + fe*r;
        end = end + ef*r;
        tempobs3 = DrawLine(first,end,OCCUPANCY,false);
        gridObstacles.insert(gridObstacles.end(),tempobs1.begin(),tempobs1.end());
        gridObstacles.insert(gridObstacles.end(),tempobs2.begin(),tempobs2.end());
        gridObstacles.insert(gridObstacles.end(),tempobs3.begin(),tempobs3.end());
    }
}

void GridMap::DrawObstacle(Vec2 first, Vec2 end) {
    vector<int> tempobs;
    tempobs = DrawLine(first,end,OCCUPANCY,false);
    gridObstacles.insert(gridObstacles.end(),tempobs.begin(),tempobs.end());
}

std::vector<int*> GridMap::DrawArc(double radius, Vec2 firstPoint, Vec2 endPoint, int value, bool stoppable) {
    Vec2 rpose(pose.x,pose.y);
    Vec2 first;
    Vec2 end;
    first = firstPoint - rpose;
    end = endPoint - rpose;
    double deltaFi = (atan2(end.y, end.x) - atan2(first.y,first.x));
    if(abs(deltaFi) > PI) {
        if(deltaFi < 0) {
            deltaFi = 2*PI + deltaFi;
        }
        else {
            deltaFi = deltaFi - 2*PI;
        }
    }
    if(deltaFi < 0) {
        Vec2 temp;
        temp = first;
        first = end;
        end = temp;
    }
    double rad = abs(deltaFi);
    DrawCircle(first,radius,rad,value,stoppable);
    vector<int*> ret;
    return ret;
}

void GridMap::DrawCircle() {
    Vec2 rpose(pose.x,pose.y);
    int num = ceil((PI*2/resolution)*senser*40);
    double dfi = PI*2/num;
    polar_point iter;
    iter.alfa = 0;
    iter.r = 100; // nem klne hogy tul nagy legyen
    vector<polar_point> circ;
    for(int i = 0; i < num + 1; i++) {
        iter.alfa = dfi*(double)i;
        iter.alfa = iter.alfa > M_PI ? iter.alfa-2.0*M_PI : iter.alfa;
        circ.push_back(iter);
    }
    vector<Point> descartcirc;
    polar2descart(circ,descartcirc);
    Vec2 t;
    for(int i = 0; i < num + 1; i++) {
        t.x = descartcirc[i].x;
        t.y = descartcirc[i].y;
        t = t + rpose;
        DrawLine(rpose,t,KNOWN,true);
    }
}

std::vector<int> GridMap::DrawCircle(Vec2 start, double radius, double rad, int value, bool stoppable) {
    Vec2 rpose(pose.x,pose.y);
    std::vector<int> ret;
    std::vector<int> tempBuf;
    int num = ceil((rad/resolution)*senser*40);
    double dfi = rad/num;
    polar_point iter = descart2polar(start);
    double base = iter.alfa;
    iter.alfa = 0;
    iter.r = radius;
    vector<polar_point> circ;
    for(int i = 0; i < num + 1; i++) {
        iter.alfa = base + dfi*(double)i;
        iter.alfa = iter.alfa > M_PI ? iter.alfa-2.0*M_PI : iter.alfa;
        circ.push_back(iter);
    }
    vector<Point> descartcirc;
    polar2descart(circ,descartcirc);
    Vec2 t;
    for(int i = 0; i < num + 1; i++) {
        t.x = descartcirc[i].x;
        t.y = descartcirc[i].y;
        t = t + rpose;
        tempBuf = DrawLine(rpose,t,value,stoppable);
        ret.insert(ret.end(),tempBuf.begin(),tempBuf.end());
    }
    return ret;
}



std::vector<int> GridMap::DrawNegativCircle(Vec2 start, double radius, double rad, int value, bool stoppable) {
    Vec2 rpose(pose.x,pose.y);
    std::vector<int> ret;
    std::vector<int> tempBuf;
    int num = ceil((rad/resolution)*senser*40);
    double dfi = rad/num;
    polar_point iter = descart2polar(start);
    double base = iter.alfa;
    iter.alfa = 0;
    iter.r = radius;
    vector<polar_point> circ;
    for(int i = 0; i < num + 1; i++) {
        iter.alfa = base - dfi*(double)i;
        iter.alfa = iter.alfa < (M_PI*(-1)) ? iter.alfa + 2.0*M_PI : iter.alfa;
        circ.push_back(iter);
    }
    vector<Point> descartcirc;
    polar2descart(circ,descartcirc);
    Vec2 t;
    for(int i = 0; i < num + 1; i++) {
        t.x = descartcirc[i].x;
        t.y = descartcirc[i].y;
        t = t + rpose;
        tempBuf = DrawLine(rpose,t,value,stoppable);
        ret.insert(ret.end(),tempBuf.begin(),tempBuf.end());
    }
    return ret;
}


void GridMap::UpgradeKnownGrid(std::vector<AncientObstacle *>& obstacles) {
    Vec2 rpose(pose.x,pose.y);
    DrawCircle();
    for(int i = 0; i < obstacles.size(); i++) {
        if(Distance(rpose,obstacles[i]->FirstUp()) < Distance(rpose,obstacles[i]->FirstDown())) {
            DrawArc(senser,obstacles[i]->FirstUp(),obstacles[i]->EndUp(),KNOWN,true);
        }
        else {
            DrawArc(senser,obstacles[i]->FirstDown(),obstacles[i]->EndDown(),KNOWN,true);
        }
    }
}

std::pair<Vec2,Vec2> GridMap::ClosestFirst(AncientObstacle * obstacle) {
    Vec2 rpose(pose.x,pose.y);
    Vec2 first;
    Vec2 end;
    Point temp;
    std::pair<Vec2,Vec2> ret;
    if(Distance(rpose,obstacle->FirstUp()) < Distance(rpose,obstacle->FirstDown())) {
        first = obstacle->FirstUp();
        end = obstacle->EndUp();
    }
    else {
        first = obstacle->FirstDown();
        end = obstacle->EndDown();
    }
    Vec2 alfaFirst = first - rpose;
    Vec2 alfaEnd = end - rpose;
    double deltaFi = atan2(alfaEnd.y, alfaEnd.x) - atan2(alfaFirst.y,alfaFirst.x);
    if(abs(deltaFi) > PI) {
        if(deltaFi < 0) {
            deltaFi = 2*PI + deltaFi;
        }
        else {
            deltaFi = deltaFi - 2*PI;
        }
    }
    if(deltaFi < 0) {
        Vec2 temp;
        temp = first;
        first = end;
        end = temp;
    }
    ret.first = first;
    ret.second = end;
    return ret;
}

int compareObsVec (const void * a, const void * b)
{
  if ( *(ObsVec*)a <  *(ObsVec*)b ) return -1;
  if ( *(ObsVec*)a == *(ObsVec*)b ) return 0;
  if ( *(ObsVec*)a >  *(ObsVec*)b ) return 1;
}

std::vector<AncientObstacle*> GridMap::SortObstacles(std::vector<AncientObstacle *> &obstacles) {
    std::vector<ObsVec> ret;
    Vec2 rpose(pose.x,pose.y);
    for(int i = 0; i < obstacles.size(); i++) {
        ret.push_back(ObsVec(obstacles[i],obstacles[i]->FirstPoint() - rpose));
    }
    qsort(&ret[0], ret.size(), sizeof(ObsVec), compareObsVec);
    vector<AncientObstacle*> sortret;
    for(int i = 0; i < ret.size(); i++) {
        sortret.push_back(ret[i].obspointer);
    }
    return sortret;
}

void GridMap::UpgradeTargets(std::vector<AncientObstacle *> &obstaclesV) {
    vector<AncientObstacle *> obstacles;
    obstacles = SortObstacles(obstaclesV);
    Vec2 rpose(pose.x,pose.y);
    Vec2 first;
    Vec2 nextfirst;
    Vec2 nextend;
    Vec2 previousEnd;
    Vec2 end;
    Vec2 start;
    Vec2 goal;
    Point temp;
    Vec2 fe;
    Vec2 ef;
    Vec2 nef;
    std::pair<Vec2,Vec2> inter;
    std::vector<int> tempBuf;
    polar_point targetStart;
    double alfa;
    for(int i = 0; i < obstacles.size(); i++) {
        alfa = atan2(obstacles[i]->FirstPoint().y,obstacles[i]->FirstPoint().x);
        inter = ClosestFirst(obstacles[i]);
        first = inter.first;
        end = inter.second;
        start = end - first;
        if(i + 1 == obstacles.size()) {
            inter = ClosestFirst(obstacles[0]);
        }
        else {
            inter = ClosestFirst(obstacles[i + 1]);
        }
        nextfirst = inter.first;
        nextend = inter.second;
        ef = end -first;;
        nef = nextend - nextfirst;
        if( Distance(end,nextfirst) <= resolution*2 ) {
            continue;
        }
        goal = nextfirst - end;
        pose.x = end.x;
        pose.y = end.y;
        start.Rotate(PI/3);
        tempBuf = DrawCircle(start,senser/3,45*PI/180,TARGET,true);
        targets.push_back(tempBuf);
        pose.x = rpose.x;
        pose.y = rpose.y;
    }
    Vec2 previousFirst;
    Vec2 pfe;
    for(int i = 0; i < obstacles.size(); i++) {
        inter = ClosestFirst(obstacles[i]);
        first = inter.first;
        end = inter.second;
        start = first - end;
        if(i - 1 < 0) {
            inter = ClosestFirst(obstacles[obstacles.size() - 1]);
        }
        else {
            inter = ClosestFirst(obstacles[i - 1]);
        }
        previousEnd = inter.second;
        previousFirst = inter.first;
        ef = end - first;
        pfe = previousFirst - previousEnd;
        if(Distance(first,previousEnd) < resolution*2) {
            continue;
        }
        pose.x = first.x;
        pose.y = first.y;
        start.Rotate(-PI/3);
        tempBuf = DrawNegativCircle(start,senser/3,45*PI/180,TARGET,true);
        targets.push_back(tempBuf);
        pose.x = rpose.x;
        pose.y = rpose.y;
    }
        for(int i = 0; i < targets.size(); i++) {
            for(int j = 0; j < targets[i].size(); j++) {
                if (data[targets[i][j]] == KNOWN) {
                    targets[i].erase(targets[i].begin() + j);
                    j--;
                }
            }
        }
}

Vec2 GridMap::MapIndexInverse(int index) {
    Vec2 eredmeny;
    int t = 0;
    eredmeny.x = index + t*mapWidth;
    eredmeny.y = 0;
    while( !(((eredmeny.x >= 0) && (eredmeny.x <= mapWidth)) && ((eredmeny.y >= 0) && (eredmeny.y <= mapWidth)))) {
        t--;
        eredmeny.x = index + t*mapWidth;
        eredmeny.y = 0 - t;
    }
    eredmeny = eredmeny - offset;
    eredmeny.x = (eredmeny.x + 0.5)*resolution;
    eredmeny.y = (eredmeny.y + 0.5)*resolution;
    return eredmeny;
}

Vec2 GridMap::QuantMapIndexInverse(int index) {
    Vec2 eredmeny;
    int t = 0;
    eredmeny.x = index + t*mapWidth;
    eredmeny.y = 0;
    while( !(((eredmeny.x >= 0) && (eredmeny.x <= mapWidth)) && ((eredmeny.y >= 0) && (eredmeny.y <= mapWidth)))) {
        t--;
        eredmeny.x = index + t*mapWidth;
        eredmeny.y = 0 - t;
    }
    return eredmeny;
}

Vec2 GridMap::NextGoal() {
   double minDist = mapWidth*2*resolution;
   double minTemp;
   Vec2 minVec;
   Vec2 tempVec;
   int targethalmaz;
   Vec2 rpose(pose.x,pose.y);
   if(targets.size() == 0 ) {
       return Vec2(0,0);
   }
   for(int i = 0; i < targets.size(); i++) {
       for(int j = 0; j < targets[i].size(); j++) {
           tempVec = MapIndexInverse(targets[i][j]);
           minTemp = Distance(rpose,tempVec);
           if(minTemp < minDist) {
                minDist = minTemp;
                minVec = tempVec;
                targethalmaz = i;
           }
       }
   }
   for(int j = 0; j < targets[targethalmaz].size(); j++) {
       data[targets[targethalmaz][j]] = KNOWN;
   }
   targets.erase(targets.begin() + targethalmaz);
   //SetGrid(Vec2Quantization(minVec),OCCUPANCY);
   return minVec;
}

void GridMap::SetRobotPose(Vec2 pose) {
    this->pose.x = pose.x;
    this->pose.y = pose.y;
}


int MapSearchNode::GetMap( int x, int y )
{
    if( x < 0 ||
        x >= gmap->mapHeight||
         y < 0 ||
         y >= gmap->mapHeight
      )
    {
        return OCCUPANCY;
    }

    return (int)gmap->data[(y*gmap->mapHeight)+x];
}

bool MapSearchNode::IsSameState( MapSearchNode &rhs )
{

    // same state in a maze search is simply when (x,y) are the same
    if( (x == rhs.x) &&
        (y == rhs.y) )
    {
        return true;
    }
    else
    {
        return false;
    }

}

void MapSearchNode::PrintNodeInfo()
{
    char str[100];
  //  sprintf( str, "Node position : (%d,%d)\n", x,y );
    cout << str;
}

void MapSearchNode::PrintNodeInfo(vector<Vec2> &path)
{
    char str[100];
   // sprintf( str, "Node position : (%d,%d)\n", x,y );
    path.push_back(Vec2(x,y));
}

// Here's the heuristic function that estimates the distance from a Node
// to the Goal.


float MapSearchNode::GoalDistanceEstimate( MapSearchNode &nodeGoal )
{
    Vec2 pose(x,y);
    Vec2 goal(nodeGoal.x,nodeGoal.y);
    Vec2 dist = pose - goal;
    return dist.Lenght();
}

bool MapSearchNode::IsGoal( MapSearchNode &nodeGoal )
{

    if( (x == nodeGoal.x) &&
        (y == nodeGoal.y) )
    {
        return true;
    }

    return false;
}

// This generates the successors to the given Node. It uses a helper function called
// AddSuccessor to give the successors to the AStar class. The A* specific initialisation
// is done for each node internally, so here you just set the state information that
// is specific to the application
bool MapSearchNode::GetSuccessors( AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node )
{
    int parent_x = -1;
    int parent_y = -1;

    if( parent_node )
    {
        parent_x = parent_node->x;
        parent_y = parent_node->y;
    }


    MapSearchNode NewNode;

    // push each possible move except allowing the search to go backwards

    if( (GetMap( x-1, y ) != OCCUPANCY)
        && !((parent_x == x-1) && (parent_y == y))
      )
    {
        NewNode = MapSearchNode( x-1, y );
        astarsearch->AddSuccessor( NewNode );
    }

    if( (GetMap( x, y-1 ) != OCCUPANCY)
        && !((parent_x == x) && (parent_y == y-1))
      )
    {
        NewNode = MapSearchNode( x, y-1 );
        astarsearch->AddSuccessor( NewNode );
    }

    if( (GetMap( x+1, y ) != OCCUPANCY)
        && !((parent_x == x+1) && (parent_y == y))
      )
    {
        NewNode = MapSearchNode( x+1, y );
        astarsearch->AddSuccessor( NewNode );
    }


    if( (GetMap( x, y+1 ) != OCCUPANCY)
        && !((parent_x == x) && (parent_y == y+1))
        )
    {
        NewNode = MapSearchNode( x, y+1 );
        astarsearch->AddSuccessor( NewNode );
    }

    if( (GetMap( x+1, y+1 ) != OCCUPANCY)
        && !((parent_x == x+1) && (parent_y == y+1))
        )
    {
        NewNode = MapSearchNode( x+1, y+1 );
        astarsearch->AddSuccessor( NewNode );
    }
    if( (GetMap( x-1, y-1 ) != OCCUPANCY)
        && !((parent_x == x-1) && (parent_y == y-1))
        )
    {
        NewNode = MapSearchNode( x-1, y-1 );
        astarsearch->AddSuccessor( NewNode );
    }

    if( (GetMap( x+1, y-1 ) != OCCUPANCY)
        && !((parent_x == x+1) && (parent_y == y-1))
        )
    {
        NewNode = MapSearchNode( x+1, y-1 );
        astarsearch->AddSuccessor( NewNode );
    }

    if( (GetMap( x-1, y+1 ) != OCCUPANCY)
        && !((parent_x == x-1) && (parent_y == y+1))
        )
    {
        NewNode = MapSearchNode( x-1, y+1 );
        astarsearch->AddSuccessor( NewNode );
    }

    return true;
}


// given this node, what does it cost to move to successor. In the case
// of our map the answer is the map terrain value at this node since that is
// conceptually where we're moving

float MapSearchNode::GetCost( MapSearchNode &successor )
{
    Vec2 pose(x,y);
    Vec2 sucPose(successor.x,successor.y);
    Vec2 d = pose - sucPose;
    if(d.x*d.y == 0) {
        return (float)1;
    }
    else {
        return (float)1.41;
    }
}

Astar::Astar(): obsbuffer() {
    vecpath.reserve(2000);
}

void Astar::Framing() {
    obsbuffer = gmap->gridObstacles;
    int x_max = 0;
    int y_max = 0;
    int y_min = gmap->mapWidth;
    int x_min = gmap->mapWidth;
    Vec2 iterator;
    for(int i = 0; i < obsbuffer.size(); i++) {
        iterator = gmap->QuantMapIndexInverse(obsbuffer[i]);
        if(x_max < iterator.x ) {
            x_max = iterator.x;
        }
        if(x_min > iterator.x) {
            x_min = iterator.x;
        }
        if(y_max < iterator.y ) {
            y_max = iterator.y;
        }
        if(y_min > iterator.y) {
            y_min = iterator.y;
        }
    }
    if(x_max < goalVec.x) {
        x_max = goalVec.x;
    }
    if(x_min > goalVec.x) {
        x_min = goalVec.x;
    }

    if(y_max < goalVec.y) {
        y_max = goalVec.y;
    }
    if(y_min > goalVec.y) {
        y_min = goalVec.y;
    }

    if(x_max < startVec.x) {
        x_max = startVec.x;
    }
    if(x_min > startVec.x) {
        x_min = startVec.x;
    }

    if(y_max < startVec.y) {
        y_max = startVec.y;
    }
    if(y_min > startVec.y) {
        y_min = startVec.y;
    }

    int board = 5;

    x_max = x_max + board;
    if(x_max > gmap->mapWidth) {
        x_max = gmap->mapWidth;
    }
    y_max = y_max + board;
    if(y_max > gmap->mapWidth) {
        y_max = gmap->mapWidth;
    }
    x_min = x_min - board;
    if(x_min < 0) {
        x_min = 0;
    }
    y_min = y_min - board;
    if(y_min < 0) {
        y_min = 0;
    }
    int XD = x_max - x_min;
    int YD = y_max - y_min;
    int iter = x_min;
    int index;
    vector<int> tempobs;
    tempobs.reserve(2*(XD+YD));
    for(int i = 0; i < XD; i++) {

        tempobs.push_back(y_min*gmap->mapWidth + iter);
        tempobs.push_back(y_max*gmap->mapWidth + iter);
        iter++;
    }
    iter = y_min;
    for(int i = 0; i < YD; i++) {

        tempobs.push_back(iter*gmap->mapWidth + x_min);
        tempobs.push_back( iter*gmap->mapWidth + x_max);
        iter++;
    }
    tempobs.push_back( y_max*gmap->mapWidth + x_max);
    gmap->gridObstacles.insert(gmap->gridObstacles.end(),tempobs.begin(),tempobs.end());
    for(int i = 0; i < tempobs.size();i++) {
        gmap->data[tempobs[i]] = OCCUPANCY;
    }
}

bool Astar::FindPath(Vec2 start, Vec2 goal) {
    vecpath.clear();
    vecpath.reserve(5000);
    AStarSearch<MapSearchNode> astarsearch;
    // Create a start state
    bool pathfound = true;
    start = gmap->Vec2Quantization(start);
    startVec = start;
    goal = gmap->Vec2Quantization(goal);
    goalVec = goal;
    Framing(); // ismert map bekeretetése
    MapSearchNode nodeStart;
    nodeStart.x = start.x;
    nodeStart.y = start.y;
    MapSearchNode nodeEnd;
    nodeEnd.x = goal.x;
    nodeEnd.y = goal.y;

    astarsearch.SetStartAndGoalStates( nodeStart, nodeEnd );

    unsigned int SearchState;
    unsigned int SearchSteps = 0;

    do
    {
        SearchState = astarsearch.SearchStep();
        if(SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_FAILED) {
            cout<<"Break "<<endl;
            break;
        }
        SearchSteps++;
#if DEBUG_LISTS

        cout << "Steps:" << SearchSteps << "\n";

        int len = 0;

        cout << "Open:\n";
        MapSearchNode *p = astarsearch.GetOpenListStart();
        while( p )
        {
            len++;
#if !DEBUG_LIST_LENGTHS_ONLY
            ((MapSearchNode *)p)->PrintNodeInfo();
#endif
            p = astarsearch.GetOpenListNext();

        }
        cout << "Open list has " << len << " nodes\n";

        len = 0;

        cout << "Closed:\n";
        p = astarsearch.GetClosedListStart();
        while( p )
        {
            len++;
#if !DEBUG_LIST_LENGTHS_ONLY
            p->PrintNodeInfo();
#endif
            p = astarsearch.GetClosedListNext();
        }

        cout << "Closed list has " << len << " nodes\n";
#endif

    }
    while( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SEARCHING );
    cout<<"Break succeseed"<<endl;
    if( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SUCCEEDED )
    {
        cout << "Search found goal state\n";

            MapSearchNode *node = astarsearch.GetSolutionStart();

#if DISPLAY_SOLUTION
            cout << "Displaying solution\n";
#endif
            int steps = 0;

            //node->PrintNodeInfo();
            for( ;; )
            {
                node = astarsearch.GetSolutionNext();
                //int x = node->x;
                //int y = node->y;
                //vecpath.push_back(Vec2(x,y));
                //gmap->data[gmap->mapHeight*node->y + node->x] = 100;
                if( !node )
                {
                    break;
                }

                node->PrintNodeInfo(vecpath);
                steps ++;

            };

            cout << "Solution steps " << steps << endl;

            // Once you're done with the solution you can free the nodes up
            astarsearch.FreeSolutionNodes();

            for(int i= 0; i < 2;i++) {
                PotencialDistort();
            }            
            PathLines();
            /*for(int i = 0; i < vecpath.size() - 1; i++) {
                gmap->DrawGridLine(vecpath[i],vecpath[i + 1]);
            }*/
           for(int i = 1; i < vecpath.size()-1;i++) {
               if((vecpath[i].x == vecpath[i + 1].x) && (vecpath[i].y == vecpath[i + 1].y)) {
                   vecpath.erase(vecpath.begin() + (i + 1));
                   i--;
               }
           }
           /*for(int i = 0; i < vecpath.size();i++) {
                gmap->data[gmap->mapHeight*(int)vecpath[i].y + (int)vecpath[i].x] = KNOWN;
            }*/
           for(int i = 0; i < vecpath.size(); i++) {
               vecpath[i] = gmap->MapIndexInverse((int)vecpath[i].y + vecpath[i].x);
           }

    }
    else if( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_FAILED )
    {
        cout << "Search terminated. Did not find goal state\n";
        pathfound = false;

    }
    // Display the number of loops the search went through
    cout << "SearchSteps : " << SearchSteps << "\n";
//---------------------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------------------
    astarsearch.EnsureMemoryFreed();
    return pathfound;
}

void Astar::SetGoalCharge() {
    Vec2 goalGrad;
    Vec2 force;
    Vec2 delta;
    Vec2 sum;
    Vec2 pose = goalVec;
    for(int i = 0; i < gmap->gridObstacles.size(); i++) {
        force = gmap->QuantMapIndexInverse(gmap->gridObstacles[i]);
        delta = pose - force;
        delta = delta.Norm()*(1/delta.Lenght());
        sum= sum + delta;
    }
    cout<<"Goal grad"<<sum<<endl;
    goalCharge = (sum.Lenght()*gmap->resolution*1.41)/2;
    cout<<"Charge "<<goalCharge<<endl;
}

Vec2 Astar::Gradient(Vec2 pose) {
    Vec2 force;
    Vec2 delta;
    Vec2 sum;
    for(int i = 0; i < gmap->gridObstacles.size(); i++) {
        force = gmap->QuantMapIndexInverse(gmap->gridObstacles[i]);
        delta = pose - force;
        delta = delta.Norm()*(1/pow(delta.Lenght(),2));
        sum= sum + delta;// + deltaGoal;
    }
    return gmap->Vec2QBaseVector(sum);
}

void Astar::PotencialDistort() {
    for(int i = 1; i < vecpath.size() - 1; i++) { // start end goal not distort
        vecpath[i] = vecpath[i] + Gradient(vecpath[i]);
    }

}

void Astar::PathLines() {
    simplifyPath pathlines;
    vector<line> lines;
    vector<polar_point> templines = descart2polar(this->vecpath);
    vector<pair<line,vector<polar_point> > > temp_result;
    temp_result = pathlines.simplifyWithRDP(templines,1.42);
    cout<<"temp "<<temp_result.size()<<endl;
    for(int i = 0; i < temp_result.size(); i++)
    {
        lines.push_back(temp_result[i].first);
    }
    cout<<"lines number "<<lines.size()<<endl;
    vecpath.clear();
    templines.clear();
    templines.reserve(lines.size()*2);
    for(int i = 0; i < lines.size(); i++) {
        templines.push_back(lines[i].lineInterval[0]);
        templines.push_back(lines[i].lineInterval[1]);
    }
    polar2descart(templines,vecpath);
}
