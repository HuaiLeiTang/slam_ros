#include "rrts.h"
#include <cmath>
#include "vec2.h"
#include <iostream>
#include <algorithm>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <fstream>
#include "dijkstra.h"
#include "lineFitting.h"

#define PI 3.141592

using namespace std;

class graph
{
public:
    int v;
    double **matrix;
    graph(int V) {
        v = V;
        matrix = new double*[V];
        for(int i = 0; i < V; i++){
            matrix[i] = new double[V];
            for(int j = 0; j < V; j++) {
                matrix[i][j] = 0;
            }
        }
    }
    void addEdge(int i,int j, double weight) {
        matrix[i][j] = weight;
        matrix[j][i] = weight;
    }

    ~graph() {
        for(int i = 0; i < v; i++) {
            delete [] matrix[i];
        }
        delete [] matrix;
    }
};

Obstacle::Obstacle(double x, double y): x(x), y(y) {
    type = pointObstacle;
}

Obstacle::Obstacle(): x(), y(){
    type = pointObstacle;
}

void Obstacle::Draw(ofstream& out) {
    out<<x<<" "<<y<<endl;
}

Vec2 Obstacle::FirstPoint() {
    return Vec2();
}

Vec2 Obstacle::EndPoint() {
    return Vec2();
}

void Obstacle::CircVecDist(Node* random, Node graf) {
    Vec2 newRand(*random,graf);
    cout<<"Circ"<<endl;
    newRand.Print();
    double R = r + 0.1;
    Vec2 PO(x,y);
    Vec2 PR(*random);
    Vec2 PG(graf);
    Vec2 GR = PR - PG;
    Vec2 RG = GR * (-1);
    Vec2 RO = PO - PR;
    Vec2 GRNorm = GR.Norm();
    newRand = newRand - (GRNorm * (R + 0.1));
    Vec2 GO = PO - PG;
    double dAlfa = SubtendedAngle(RG,RO,GO);
    if(dAlfa > (90 * PI/180)) {
        double dBeta = 180 * PI/180 - dAlfa;
        newRand = newRand + GRNorm*((R - sqrt(pow(R,2) - pow(sin(dBeta)*RO.Lenght(),2))) + RO.Lenght()*cos(dBeta));
        newRand = newRand + PG;
        random->x = newRand.x;
        random->y = newRand.y;
        return;
    }
    else {
        newRand = newRand + GRNorm*((r - sqrt(pow(r,2) - pow(sin(dAlfa)*RO.Lenght(),2))) - RO.Lenght()*cos(dAlfa));
        newRand = newRand + PG;
        random->x = newRand.x;
        random->y = newRand.y;
        return;
    }
}

double sgn(double sign) {
    if(sign < (90*PI/180)) {
        return 1;
    }
    else {
        return -1;
    }
}

bool Obstacle::IsCollision(Node inGraf, Node *randNode,bool edit) {
    Vec2 F(inGraf);
    Vec2 O(x,y);
    Vec2 R(*randNode);
    Vec2 RF = F - R;
    Vec2 RO = O - R;
    Vec2 FO = O - F;
    Vec2 newNode;
    double cosAlfa = SubtendedCos(FO,RF,RO);
    double cosBeta = SubtendedCos(RO,RF,FO);
    double sinAlfa = sqrt(1 - pow(cosAlfa,2));
    double sinBeta = sqrt(1 -pow(cosBeta,2));
    if(edit) {
        if((((RO.Lenght() <= r) && (cosBeta < 0) ) || (( sinAlfa*FO.Lenght() <= r) && (cosBeta > 0))) && (cosAlfa > 0)) {
            newNode = R + RF.Norm()*(r + RO.Lenght()*cosBeta - (r - sqrt(pow(r,2) - pow(sinBeta*RO.Lenght(),2))));
            randNode->x = newNode.x;
            randNode->y = newNode.y;
            return true;
        }
        return false;
    }
    else {
        if((((RO.Lenght() <= r*0.99) && (cosBeta < 0) ) || (( sinAlfa*FO.Lenght() <= r*0.99) && (cosBeta > 0))) && (cosAlfa > 0)) {
            return true;
        }
        return false;
    }
}

StraightObstacle::StraightObstacle():firstPoint(), endPoint() {
    type = lineObstacle;
}

StraightObstacle::StraightObstacle(Vec2 firstPoint, Vec2 endpoint) {
    this->firstPoint = firstPoint;
    this->endPoint = endpoint;
    type = lineObstacle;
    Vec2 EF = endpoint - firstPoint;
    Vec2 EF_norm = EF.Norm();
    Vec2 FE_nrom = EF_norm*(-1);
    Vec2 UF = EF_norm;
    UF.RoundPoz90();
    Vec2 DF = EF_norm;
    DF.RoundMinusz90();
    Vec2 UE = FE_nrom;
    UE.RoundMinusz90();
    Vec2 DE = FE_nrom;
    DE.RoundPoz90();
    this->firstUp = firstPoint + UF*r + FE_nrom*r;
    this->firstDown = firstPoint + DF*r + FE_nrom*r;
    this->endUp = endPoint + UE*r + EF_norm*r;
    this->endDown = endPoint + DE*r + EF_norm*r;
}

StraightObstacle::StraightObstacle(polar_point firstPoint, polar_point endPoint) {
    vector<Point> tempPoints;
    vector<polar_point> tempPolars;
    tempPolars.push_back(firstPoint);
    tempPolars.push_back(endPoint);
    tempPoints = polar2descart(tempPolars);
    Vec2 f(tempPoints[0].x*100, tempPoints[0].y*100);
    Vec2 e(tempPoints[1].x*100, tempPoints[1].y*100);
    this->firstPoint = f;
    this->endPoint = e;
    type = lineObstacle;
    Vec2 EF = this->endPoint - this->firstPoint;
    Vec2 EF_norm = EF.Norm();
    Vec2 FE_nrom = EF_norm*(-1);
    Vec2 UF = EF_norm;
    UF.RoundPoz90();
    Vec2 DF = EF_norm;
    DF.RoundMinusz90();
    Vec2 UE = FE_nrom;
    UE.RoundMinusz90();
    Vec2 DE = FE_nrom;
    DE.RoundPoz90();
    this->firstUp = this->firstPoint + UF*r + FE_nrom*r;
    this->firstDown = this->firstPoint + DF*r + FE_nrom*r;
    this->endUp = this->endPoint + UE*r + EF_norm*r;
    this->endDown = this->endPoint + DE*r + EF_norm*r;
}


Vec2 StraightObstacle::FirstPoint() {
    return firstPoint;
}

Vec2 StraightObstacle::EndPoint() {
    return endPoint;
}

Vec2 StraightObstacle::FirstUp() {
    return firstUp;
}

Vec2 StraightObstacle::FirstDown() {
    return firstDown;
}

Vec2 StraightObstacle::EndUp() {
    return endUp;
}

Vec2 StraightObstacle::EndDown() {
    return endDown;
}

void StraightObstacle::Draw(ofstream &out) {
    out<<firstPoint<<endl;
    out<<endPoint<<endl;
}

bool StraightObstacle::Between(Vec2 m) {
    double downX;
    double upX;
    double downY;
    double upY;
    if(firstPoint.x >= endPoint.x) {
        upX = firstPoint.x;
        downX = endPoint.x;
    }
    else {
        upX = endPoint.x;
        downX = firstPoint.x;
    }

    if(firstPoint.y >= endPoint.y) {
        upY = firstPoint.y;
        downY = endPoint.y;
    }
    else {
        upY = endPoint.y;
        downY = firstPoint.y;
    }
    bool xval;
    bool yval;
    if( ( m.x>= downX ) && ( m.x <= upX)) {
            xval = true;
    }
    else {
        return false;
    }
    if( (m.y >= downY) && (m.y <= upY) ) {
        yval = true;
    }
    else {
        return false;
    }
    return (xval && yval);
}

bool NodeBetween(Vec2 a, Vec2 b, Vec2 m) {
    double upX;
    double downX;
    double upY;
    double downY;
    if(a.x >= b.x) {
        upX = a.x;
        downX = b.x;
    }
    else {
        upX = b.x;
        downX = a.x;
    }

    if(a.y >= b.y) {
        upY = a.y;
        downY = b.y;
    }
    else {
        upY = b.y;
        downY = a.y;
    }
    bool xval;
    bool yval;
    if((m.x >= (downX-0.1)) && ( m.x <= (upX + 0.1))) {
            xval = true;
    }
    else {
        return false;
    }

    if( (m.y >= (downY - 0.1)) && (m.y <= (upY + 0.1)) ) {
        yval = true;
    }
    else {
        return false;
    }
    return (xval && yval);
}

Vec2 IntersectionPoint(Vec2 a, Vec2 b, Vec2 c, Vec2 d) {
    Vec2 n_1;
    Vec2 n_2;
    n_2 = c - d;
    n_1 = a - b;
    n_1.Merolegese();
    n_2.Merolegese();
    double A[4];
    A[0] = n_1.x; // |0 2|
    A[1] = n_2.x; // |1 3|
    A[2] = n_1.y;
    A[3] = n_2.y;
    double B[2];
    B[0] = n_1.x*a.x + n_1.y*a.y;
    B[1] = n_2.x*c.x + n_2.y*c.y;
    double det =1/(A[0]*A[3] - A[2]*A[1]);
    double A_inv[4];
    A_inv[0] = det*A[3];
    A_inv[3] = det*A[0];
    A_inv[1] = det*-A[1];
    A_inv[2] = det*-A[2];
    Vec2 m;
    m.x = A_inv[0]*B[0] + A_inv[2]*B[1];
    m.y = A_inv[1]*B[0] + A_inv[3]*B[1];
    return m;
}

bool StraightObstacle::IsCollision(Node inGraf, Node *randNode, bool edit) {
    Vec2 G(inGraf);
    Vec2 R(*randNode);
    vector<Vec2> metszesek;
    Vec2 m;
    m = IntersectionPoint(firstDown,firstUp,G,R);
    if(NodeBetween(G,R,m) && NodeBetween(firstDown,firstUp,m)) {
        metszesek.push_back(m);
    }
    m = IntersectionPoint(firstUp, endUp, G, R);
    if(NodeBetween(G,R,m) && NodeBetween(firstUp,endUp,m)) {
        metszesek.push_back(m);
    }
    m = IntersectionPoint(endUp,endDown,G,R);
    if(NodeBetween(G,R,m) && NodeBetween(endUp,endDown,m)) {
        metszesek.push_back(m);
    }
    m = IntersectionPoint(endDown, firstDown,G,R);
    if( NodeBetween(G,R,m) && NodeBetween(endDown,firstDown,m)) {
        metszesek.push_back(m);
    }
    if(metszesek.size() == 0) {
        return false;
    }
    else {
        if(!edit) {
            return true;
        }
    }
    if(edit) {
        double minDist;
        double tempDist;
        Vec2 minVec;
        minDist = Distance(metszesek[0],G);
        minVec = metszesek[0];
        for(int i = 1; i < metszesek.size(); i++) {
            tempDist = Distance(metszesek[i],G);
            if(tempDist < minDist) {
                minVec = metszesek[i];
            }
        }
        randNode->x = minVec.x;
        randNode->y = minVec.y;
        return true;
    }

}

RRTs::RRTs(std::vector<AncientObstacle*> map,Node firstNode ,double maxX, double maxY): maxMapSizeX(maxX),
    maxMapSizeY(maxY), graf(), path(), reducedPath(), sendPath(), dijkPath(), visibleGraft() {
    obstacles = map;
    Node* first = new Node[1];
    *first = firstNode;
    graf.push_back(first);
    srand (time(NULL));
}

RRTs::RRTs(double maxX, double maxY): maxMapSizeX(maxX),
    maxMapSizeY(maxY), graf(), path(), reducedPath(), sendPath(), dijkPath(), visibleGraft() {
    srand (time(NULL));
    graf.reserve(10000);
    path.reserve(2000);
    reducedPath.reserve(2000);
    sendPath.reserve(1000);
    dijkPath.reserve(2000);
    visibleGraft.reserve(2000);
}


bool RRTs::IsPartOfGraf(Node *newNode) {
   return newNode->partOf;
}

void RRTs::InsertToGraf(Node *newNode) {// elötte isPartOfGraf-fal ellenörnizni, hogy eleme-e
    graf.push_back(newNode);
    newNode->partOf = true;
    if(newNode->IsLeaf()) {
        newNode->parent->childern.push_back(newNode);
    }
    else {
        newNode->parent->childern.push_back(newNode);
        std::vector<Node*>::iterator it;
        it = find(newNode->parent->childern.begin(),newNode->parent->childern.end(),newNode->childern[0]);
        if(it == newNode->parent->childern.end()) {
            cout<<newNode->parent->childern.size()<<endl;
            cout<<newNode->childern.size()<<endl;
        }
        newNode->parent->childern.erase(it);
    }
}


Node* RRTs::ClosestNode(Node *randNode) {
    std::pair<Node*,double> newNodeCandidate;
    Node * minNode = graf[0];
    double minDist = sqrt(pow(maxMapSizeX,2) + pow(maxMapSizeY,2));
    for(int i = 0; i < graf.size(); i++) {
        for(int j = 0; j < graf[i]->childern.size(); j++) {
            newNodeCandidate = DistFromGraf(graf[i], graf[i]->childern[j],randNode);
            if(newNodeCandidate.second < minDist) {
                minDist = newNodeCandidate.second;
                minNode = newNodeCandidate.first;
            }
        }
    }
    return minNode;
}

Node* RRTs::RandNode() {
    Node* randNode;
    int choice = rand() % 100;
    if( choice < 80) {
        randNode = new Node;
        randNode->x = (double)((rand() % (int)(200*maxMapSizeX)) - (100*maxMapSizeX))/100;
        randNode->y = (double)((rand() % (int)(200*maxMapSizeY)) - (100*maxMapSizeY))/100;
    }
    else {
        randNode = new Node(goal.x,goal.y);
    }
}

void RRTs::PathPlaning(Node goal) {
    this->goal = goal;
    Node* randNode;
    Node* closestNode;
    Vec2* temprand;
    int i = 0;
    int t = 1;
    do {
        i++;
        if(i == 1000) {
            i = 0;
            t++;
            cout<<"Path planning ciklus: "<<t<<"000"<<endl;
            if(t == 4) {
               dijkPath.clear();
               return;
            }
        }
        randNode = RandNode();
        closestNode = ClosestNode(randNode);
        if(!IsPartOfGraf(closestNode)) {
            InsertToGraf(closestNode);
        }
        randNode->parent = closestNode;
        for(int i = 0; i < obstacles.size(); i++) {
            if(obstacles[i]->IsCollision(*closestNode,randNode)) {
            }
        }
        InsertToGraf(randNode);
    }while( !((randNode->x == goal.x) && (randNode->y == goal.y)));
    Node* iter = randNode;
    while(iter != NULL){
        path.push_back(iter);
        iter = iter->parent;
    }
    /*-----Dijkstra path search-----*/
    graph g(path.size());
    bool collide = false;
    int c = 0;
    Vec2 tempvisible_1;
    Vec2 tempvisible_2;
    for(int i = 0; i < path.size() - 1; i++) {
        for(int j = i + 1; j < path.size(); j++) {
            for(int k = 0; k < obstacles.size(); k++) {
                if( obstacles[k]->IsCollision(*path[i],path[j],false) ) {
                    collide = true;
                    break;
                }
            }
            if(!collide) {
                g.addEdge(i,j,Distance(path[i],path[j]));
                tempvisible_1.x = path[i]->x;
                tempvisible_1.y = path[i]->y;
                tempvisible_2.x = path[j]->x;
                tempvisible_2.y = path[j]->y;
                visibleGraft.push_back(tempvisible_1);
                visibleGraft.push_back(tempvisible_2);
                c++;
            }
            collide = false;
        }
    }
    vector<int> p;
    p = dijkstra(g.matrix,0,g.v);
    for(int i = 0; i < p.size(); i++) {
        dijkPath.push_back(*path[p[i]]);
    }
}

void RRTs::ExportGraf() {
    ofstream myfile;
    ofstream mapfile;
    ofstream linefile;
    ofstream pathfile;
    ofstream reducedPathfile;
    ofstream sendPathfile;
    ofstream dijkstrafile;
    ofstream visiblefile;
    dijkstrafile.open("/home/mono/path/dijkstra.txt");
    myfile.open("/home/mono/path/graf.txt");
    mapfile.open("/home/mono/path/map.txt");
    pathfile.open("/home/mono/path/path.txt");
    linefile.open("/home/mono/path/line.txt");
    reducedPathfile.open("/home/mono/path/reducedpath.txt");
    sendPathfile.open("/home/mono/path/sendpath.txt");
    visiblefile.open("/home/mono/path/visible.txt");
     for(int i = 0; i < graf.size(); i++) {
         for(int j = 0; j < graf[i]->childern.size(); j++) {
                 myfile<<graf[i]->x<<" "<<graf[i]->y<<endl;
                 myfile<<graf[i]->childern[j]->x<<" "<<graf[i]->childern[j]->y<<endl;
             }
     }
     for(int i = 0; i < obstacles.size(); i++) {
         if(obstacles[i]->type == pointObstacle) {
            obstacles[i]->Draw(mapfile);
         }
         if(obstacles[i]->type == lineObstacle) {
             obstacles[i]->Draw(linefile);
         }
     }
     for(int i = 0; i < path.size(); i++) {
         pathfile<<path[i]->x<<" "<<path[i]->y<<endl;
     }
     for(int i = 0; i < visibleGraft.size() - 1; i++) {
         visiblefile<<visibleGraft[i]<<endl;
         visiblefile<<visibleGraft[i+1]<<endl;

     }
     for(int i = 0; i < reducedPath.size(); i++) {
         reducedPathfile<<reducedPath[i].x<<" "<<reducedPath[i].y<<endl;
     }
     for(int i = 0; i < sendPath.size(); i++) {
         sendPathfile<<sendPath[i].x<<" "<<sendPath[i].y<<endl;
     }
     for(int i = 0; i < dijkPath.size(); i++) {
         dijkstrafile<<dijkPath[i].x<<" "<<dijkPath[i].y<<endl;
     }
}

void RRTs::Reset() {
    this->dijkPath.clear();
    this->graf.clear();
    this->path.clear();
    this->reducedPath.clear();
    this->sendPath.clear();
    graf.reserve(10000);
    path.reserve(2000);
    reducedPath.reserve(2000);
    sendPath.reserve(1000);
    dijkPath.reserve(2000);
    visibleGraft.reserve(2000);
}

void RRTs::SetPose(Vec2 pose) {
    Node* first = new Node(pose.x,pose.y);
    first->partOf = true;
    graf.push_back(first);
}

void RRTs::AddObstacles(std::vector<AncientObstacle *> newobs) {
    this->obstacles.insert(obstacles.end(),newobs.begin(),newobs.end());
}
