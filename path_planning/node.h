#ifndef NODE_H
#define NODE_H

#include <vector>
#include <utility>


class Node {
public:
    Node();
    Node(double x, double y);
    Node(double x, double y,Node* parent);
    double Distance(Node other);
    double x;
    double y;
    Node* parent;
    std::vector<Node*> childern;
    bool IsLeaf();
    bool partOf;
};

std::pair <Node*,double> DistFromGraf(Node* parent, Node* child, Node* rand);
double Distance(Node* a, Node* b);

#endif // NODE_H
