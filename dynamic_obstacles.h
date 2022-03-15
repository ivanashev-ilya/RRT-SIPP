#ifndef DYNAMICOBSTACLES_H
#define DYNAMICOBSTACLES_H
#include "gl_const.h"
#include <node.h>
#include <vector>
#include "tinyxml2.h"
#include <string>
#include <iostream>
#include <math.h>

struct obstacle
{
    std::string id;
    double size;
    double mspeed;
    std::vector<Node> sections;
    obstacle(){ id = -1; size = 1; mspeed = 1; }
};

class DynamicObstacles
{
    std::vector<obstacle> obstacles;
public:
    DynamicObstacles();
    bool getObstacles(const char* fileName);
    std::vector<Node> getSections(int num) const;
    double getSize(int num) const;
    double getMSpeed(int num) const;
    std::string getID(int num) const;
    int getNumberOfObstacles() const { return obstacles.size(); }
};

#endif // DYNAMICOBSTACLES_H
