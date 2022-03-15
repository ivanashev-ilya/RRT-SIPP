#ifndef NODE_H
#define NODE_H

#include "gl_const.h"
#include <unordered_set>

struct Node
{
    double  i, j;
    double  g;
    double  startTime = 0;
    double  endTime = CN_INFINITY;
    Node    *parent;
    std::unordered_set<Node*> successors;
    int     pointId;

    Node(double x = 0, double y = 0, Node *p = nullptr, double g_ = 0) {
        i = x;
        j = y;
        parent = p;
        g = g_;
    }
};

#endif
