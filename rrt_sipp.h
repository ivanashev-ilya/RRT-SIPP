#ifndef RRT_SIPP_H
#define RRT_SIPP_H

#include "rrt.h"

class RRT_SIPP : public RRT
{
public:
    RRT_SIPP(double Delta) : RRT(Delta) {}
    virtual ~RRT_SIPP() {}

    void getNodeVersions(const Map& map, const ConflictAvoidanceTable& CAT,
        const Node &curNode, std::vector<Node>& versions) override;

    double getRequiredWaitingTime(const Map& map, const ConflictAvoidanceTable& CAT,
        const Node& prev, const Node& cur) const override;

};

#endif // RRT_SIPP_H
