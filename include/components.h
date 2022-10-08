#ifndef COMPONENTS_H
#define COMPONENTS_H
#include <set>
#include <unistd.h>
#include <stdlib.h>
#include <set>
#include <map>
#include <eigen3/Eigen/SVD>
#include <eigen3/Eigen/Geometry>


//=================== Node =========================================
class Node
{
public:
    //-------- variabili -----------------------
    int dimension;
    float S[2];
    Node* parent;

    //-------- funzioni ------------------------
    Node(float x, float y);
};


//=================== Tree ========================================
class Tree
{
public:
    //-------- variabili -----------------------
    float r;
    std::set<Node*> V;
    std::set<Node*> Qv;
    std::set<std::pair<Node*, Node*>> E;
    std::set<std::pair<Node*, Node*>> Qe;
    std::set<Node*> Vold;
};



#endif // COMPONENTS_H
