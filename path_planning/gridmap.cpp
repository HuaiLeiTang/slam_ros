#include "gridmap.h"

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
    if( (data[MapIndex(grid)] == KNOWN) || (data[MapIndex(grid)] == OCCUPANCY)) {
        return;
    }
    data[MapIndex(grid)] = value;
}

std::vector<int*> GridMap::DrawLine(Vec2 firstPoint, Vec2 endPoint, int value, bool stoppable) {
    Vec2 kvantF = Vec2Quantization(firstPoint);
    Vec2 kvantE = Vec2Quantization(endPoint);
    Vec2 compass;
    while(true) {
        SetGrid(kvantF,value);
        compass = kvantE - kvantF;
        compass = Vec2QBaseVector(compass);
        kvantF = kvantF + compass;
        if(kvantF == kvantE) {
            break;
        }
        if(stoppable && (data[MapIndex(kvantF)] == OCCUPANCY)) {
            break;
        }
    }
    vector<int*> buf;
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
    cout<<"first "<<first<<endl;
    cout<<"rad "<<deltaFi*180/PI<<endl;
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
    iter.r = senser/3;
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

void GridMap::DrawCircle(Vec2 start, double radius, double rad, int value, bool stoppable) {
    Vec2 rpose(pose.x,pose.y);
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
        DrawLine(rpose,t,value,stoppable);
    }
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
    first = first;
    end = end;
    double deltaFi = atan2(end.y, end.x) - atan2(first.y,first.x);
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

void GridMap::UpgradeTargets(std::vector<AncientObstacle *> &obstacles) {
    Vec2 rpose(pose.x,pose.y);
    Vec2 first;
    Vec2 nextfirst;
    Vec2 previousEnd;
    Vec2 end;
    Vec2 start;
    Vec2 goal;
    Point temp;
    Vec2 fe;
    std::pair<Vec2,Vec2> inter;
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
        goal = nextfirst - end;
        pose.x = end.x;
        pose.y = end.y;
     /*   fe = first - end;
        targetStart = descart2polar(fe);
        cout<<"alfa start: "<<targetStart.alfa*180/PI <<endl;
        targetStart.alfa = targetStart.alfa - 30*PI/180;
        targetStart.alfa = targetStart.alfa > M_PI ? targetStart.alfa -2.0*M_PI : targetStart.alfa;
        targetStart.alfa = targetStart.alfa < -M_PI ? targetStart.alfa + 2.0*M_PI : targetStart.alfa;
        cout<<"alfa start uj : "<<targetStart.alfa*180/PI <<endl;
        temp = polar2descart(targetStart);
        start.x = temp.x;
        start.y = temp.y;
        cout<<"alfa "<<targetStart.alfa*180/PI <<endl;
        targetStart.alfa = targetStart.alfa - 100*PI/180;
        targetStart.alfa = targetStart.alfa > M_PI ? targetStart.alfa - 2.0*M_PI : targetStart.alfa;
        targetStart.alfa = targetStart.alfa < -M_PI ? targetStart.alfa + 2.0*M_PI : targetStart.alfa;
        cout<<"alfa uj "<<targetStart.alfa*180/PI<<endl;
        temp = polar2descart(targetStart);
        goal.x = temp.x;
        goal.y = temp.y;*/
        DrawCircle(start,senser/3,160*PI/180,TARGET,true);
        pose.x = rpose.x;
        pose.y = rpose.y;
    }

    for(int i = 0; i < obstacles.size(); i++) {
        inter = ClosestFirst(obstacles[i]);
        first = inter.first;
        end = inter.second;
        start = end - first;
        if(i - 1 < 0) {
            inter = ClosestFirst(obstacles[obstacles.size() - 1]);
        }
        else {
            inter = ClosestFirst(obstacles[i - 1]);
        }
        previousEnd = inter.second;
        pose.x = first.x;
        pose.y = first.y;
     /*   fe = first - end;
        targetStart = descart2polar(fe);
        cout<<"alfa start: "<<targetStart.alfa*180/PI <<endl;
        targetStart.alfa = targetStart.alfa - 30*PI/180;
        targetStart.alfa = targetStart.alfa > M_PI ? targetStart.alfa -2.0*M_PI : targetStart.alfa;
        targetStart.alfa = targetStart.alfa < -M_PI ? targetStart.alfa + 2.0*M_PI : targetStart.alfa;
        cout<<"alfa start uj : "<<targetStart.alfa*180/PI <<endl;
        temp = polar2descart(targetStart);
        start.x = temp.x;
        start.y = temp.y;
        cout<<"alfa "<<targetStart.alfa*180/PI <<endl;
        targetStart.alfa = targetStart.alfa - 100*PI/180;
        targetStart.alfa = targetStart.alfa > M_PI ? targetStart.alfa - 2.0*M_PI : targetStart.alfa;
        targetStart.alfa = targetStart.alfa < -M_PI ? targetStart.alfa + 2.0*M_PI : targetStart.alfa;
        cout<<"alfa uj "<<targetStart.alfa*180/PI<<endl;
        temp = polar2descart(targetStart);
        goal.x = temp.x;
        goal.y = temp.y;*/
        DrawCircle(start,senser/3,160*PI/180,TARGET,true);
        pose.x = rpose.x;
        pose.y = rpose.y;
    }
}
