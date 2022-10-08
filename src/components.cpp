#include "components.h"

//============= Node =====================================

//costruttore
Node::Node(float x, float y){
    this->S[0] = x;
    this->S[1] = y;
    this->parent = nullptr;
}
