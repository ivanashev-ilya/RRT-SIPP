#ifndef RRT_H
#define RRT_H

#include "ilogger.h"
#include "searchresult.h"
#include "environmentoptions.h"
#include "config.h"
#include "lineofsight.h"
#include "conflict_avoidance_table.h"
#include "lineofsight.h"
#include <list>
#include <vector>
#include <math.h>
#include <limits>
#include <chrono>
#include <set>
#include <map>
#include <algorithm>
#include <random>

class RRT
{
public:
    RRT(double Delta);
    virtual ~RRT();

    SearchResult startSearch(Map &map, const Config &config, const ConflictAvoidanceTable& CAT, std::mt19937 &rng);

protected:
    void makePath(const Node &curNode, const Node* nextNode = nullptr);//Makes path using back pointers
    static double distance(const Node& lhs, const Node& rhs);
    virtual bool earlyStop(const Node& cur, const Node& goal, const Map& map) const;
    bool checkObstacles(const Node& cur, const Node& neigh, const Map& map) const;
    virtual void getNodeVersions(const Map& map, const ConflictAvoidanceTable& CAT,
        const Node &curNode, std::vector<Node>& versions);

    virtual double getRequiredWaitingTime(const Map& map, const ConflictAvoidanceTable& CAT,
        const Node& prev, const Node& cur) const { return 0; }

    virtual void addNode(Node& node, const std::list<Node>::iterator& parentIt,
        const Map& map, const ConflictAvoidanceTable& CAT);

    virtual void setParent(Node& node, Node* parent) const;

    SearchResult                    sresult;
    std::list<Node>                 path;
    std::list<Node>                 nodes;
    double                          delta;
};

#endif // RRT_H
