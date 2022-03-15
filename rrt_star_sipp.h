#ifndef RRT_STAR_SIPP_H
#define RRT_STAR_SIPP_H

#include "rrt_sipp.h"

class RRT_STAR_SIPP : public RRT_SIPP
{
public:
    RRT_STAR_SIPP(double Delta) : RRT_SIPP(Delta) {}
    virtual ~RRT_STAR_SIPP() {}

    void addNode(Node& node, const std::list<Node>::iterator& parentIt,
        const Map& map, const ConflictAvoidanceTable& CAT) override;

    void setParent(Node& node, Node* parent) const override;

    void updateSubtree(Node* node, const Map& map, const ConflictAvoidanceTable& CAT);
};

#endif // RRT_STAR_SIPP_H
