#include "gridmap.h"

ObsVec::ObsVec(AncientObstacle *obs, Vec2 firstPoint): firstPoint(firstPoint) {
    obspointer = obs;
}

inline bool operator==(const ObsVec& va, const ObsVec& vb) {
        return false;
}

bool operator<(const ObsVec& va, const ObsVec& vb)
{
    double alfa = atan2(va.firstPoint.y,va.firstPoint.x);
    double beta = atan2(vb.firstPoint.y,vb.firstPoint.x);
    if((alfa + PI) > (alfa + PI))
        return true;
    else
        return false;
}

bool operator>(const ObsVec& va, const ObsVec& vb)
{
    double alfa = atan2(va.firstPoint.y,va.firstPoint.x);
    double beta = atan2(vb.firstPoint.y,vb.firstPoint.x);
    if((alfa + PI) > (alfa + PI))
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

void GridMap::SetGrid(Vec2 grid, int value) {
    if(value == OCCUPANCY) {
        data[MapIndex(grid)] = value;
        return;
    }
    if( (data[MapIndex(grid)] == KNOWN) || (data[MapIndex(grid)] == OCCUPANCY)) {
        return;
    }
    data[MapIndex(grid)] = value;
}

std::vector<int> GridMap::DrawLine(Vec2 firstPoint, Vec2 endPoint, int value, bool stoppable) {
 //   Vec2 kvantF = Vec2Quantization(firstPoint);
 //   Vec2 kvantE = Vec2Quantization(endPoint);
    Vec2 F = firstPoint;
    Vec2 kvantF;
    Vec2 FE_norm = endPoint - firstPoint;
    FE_norm = FE_norm.Norm();
    FE_norm = FE_norm * (r/2);
    if(value == TARGET) {
        F = F + FE_norm*5;
    }
 //   Vec2 compass;
    vector<int> buf;
    while(true) {
        kvantF = Vec2Quantization(F);
        SetGrid(kvantF,value);
        buf.push_back(MapIndex(kvantF));
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
    for(int i = 0; i < obstacles.size(); i++) {
        first = obstacles[i]->FirstUp();
        end = obstacles[i]->EndUp();
        DrawLine(first,end,OCCUPANCY,false);
        first = obstacles[i]->FirstDown();
        end = obstacles[i]->EndDown();
        DrawLine(first,end,OCCUPANCY,false);
        first = obstacles[i]->FirstPoint();
        end = obstacles[i]->EndPoint();
        Vec2 fe = first - end;
        fe = fe.Norm();
        Vec2 ef = fe*(-1);
        first = first + fe*r;
        end = end + ef*r;
        DrawLine(first,end,OCCUPANCY,false);
    }
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
    iter.r = 1.5; // nem klne hogy tul nagy legyen
    vector<polar_point> circ;
    for(int i = 0; i < num + 1; i++) {
        iter.alfa = PI/4 + dfi*(double)i;
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
    cout<<"end alfa: "<<atan2(alfaEnd.y, alfaEnd.x)*180/PI<<" first :"<<atan2(alfaFirst.y,alfaFirst.x)*180/PI<<endl;
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
    for(int i = 0; i < obstacles.size(); i++) {
        ret.push_back(ObsVec(obstacles[i],obstacles[i]->FirstPoint()));
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
    for(int i = 0; i < obstacles.size(); i++) {
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
        if( Distance(end,nextfirst) <= r*2 ) {
            continue;
        }
        goal = nextfirst - end;
        pose.x = end.x;
        pose.y = end.y;
        start = start;
        tempBuf = DrawCircle(start,senser/3,120*PI/180,TARGET,true);
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
        if(Distance(first,previousEnd) < r*2) {
            continue;
        }
        pose.x = first.x;
        pose.y = first.y;
        tempBuf = DrawNegativCircle(start,senser/3,120*PI/180,TARGET,true);
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
