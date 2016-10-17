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
    cout<<kvantE<< " kvant "<<kvantF<<endl;
    cout<<"sdf"<< (kvantF != kvantE) <<endl;
    cout<<"wtf"<< (kvantF != kvantE || ((data[MapIndex(kvantF)] != OCCUPANCY) && stoppable))<<endl;
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
    double rad = SubtendedAngle(rpose,firstPoint,endPoint);
    int num = ceil((rad/resolution)*radius);
    double dfi = rad/num;
    vector<polar_point> temppolar;
    temppolar.resize(num + 1);
    vector<int*> ret;
    return ret;

}

void GridMap::DrawCircle() {
    Vec2 rpose(pose.x,pose.y);
    int num = ceil((2*PI/resolution)*senser*10);
    double dfi = 2*PI/num;
    polar_point iter;
    iter.alfa = 0;
    iter.r = senser;
    vector<polar_point> circ;
    for(int i = 0; i < num + 1; i++) {
        iter.alfa = dfi* (double)i;
        iter.alfa = iter.alfa > M_PI ? iter.alfa-2.0*M_PI : iter.alfa;
        circ.push_back(iter);
    }
    cout<<"circ generate"<<endl;
    vector<Point> descartcirc;
    polar2descart(circ,descartcirc);
    cout<<"asdasdasd"<<descartcirc[num].x<<endl;
    Vec2 t;
    for(int i = 0; i < num + 1; i++) {
        t.x = descartcirc[i].x;
        t.y = descartcirc[i].y;
        t = t + rpose;
        cout<<"line "<<i<<endl;
        cout<<"t: "<<t<<" rpose: "<<rpose<<endl;
        DrawLine(rpose,t,KNOWN,true);
    }
}
