#include "node.h"
#include <cstddef>
#include <cmath>
#include "vec2.h"
#include <iostream>
#include <algorithm>

#define PI 3.141592

Node::Node(): x(0), y(0), parent(NULL), childern(), partOf(false) {

}

Node::Node(double x, double y): x(x), y(y), parent(NULL), childern(), partOf(false) {

}

Node::Node(double x, double y, Node *parent): x(x), y(y), parent(parent), childern(), partOf(true) {
    parent->childern.push_back(this);
}

double Node::Distance(Node other) {
    return sqrt(pow(this->x - other.x,2) + pow(this->y - other.y,2));
}

bool Node::IsLeaf() {
    if(childern.size() == 0) {
        return true;
    }
    else {
        return false;
    }
}

using namespace std;

std::pair<Node*,double> DistFromGraf(Node* parent, Node* child, Node* rand) { // elenörizni, hogy a kapot Node uj node vagy pedig az eddig graf eleme-e
    std::pair<Node*,double> ret;
    Vec2 PF(*parent);
    Vec2 PC(*child);
    Vec2 PR(*rand);
    Vec2 FC = PC - PF;
    Vec2 CF = FC*(-1);
    Vec2 FR = PR - PF;
    Vec2 CR = PR - PC;
    double cosAlfa = SubtendedCos(FC,FR,CR);
    double cosBeta = SubtendedCos(CF,CR,FR);
    if(cosAlfa <= 0) {
        ret.first = parent;
        ret.second = FR.Lenght();
        return ret;
    }
    if(cosBeta <= 0) {
        ret.first = child;
        ret.second = CR.Lenght();
        return ret;
    }
    double sinAlfa = sqrt(1 - pow(cosAlfa,2));
    Vec2 FCNorm = FC.Norm();
    Vec2 FN = FCNorm*(cosAlfa*FR.Lenght());
    Vec2 PN = PF + FN;
    Node* newNode = new Node;
    newNode->x = PN.x;
    newNode->y = PN.y;
    newNode->parent = parent;
    newNode->childern.push_back(child);
    ret.first = newNode;
    ret.second = sinAlfa*FR.Lenght();
    return ret;
}

double Distance(Node* a, Node* b) {
    Vec2 c(*a,*b);
    return c.Lenght();
}
