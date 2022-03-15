#ifndef CONFLICTAVOIDANCETABLE_H
#define CONFLICTAVOIDANCETABLE_H

#include <unordered_map>
#include <list>
#include "node.h"
#include "map.h"
#include <map>
#include <boost/icl/interval_set.hpp>
#include "lineofsight.h"

class ConflictAvoidanceTable
{
public:
    void addInterval(int i, int j, double start, double end);

    void addObstaclePath(const std::vector<Node>& nodes, const Map& map);

    int getFirstSoftConflict(const Node & node, int startTime, int endTime);

    void getSafeIntervals(std::vector<std::pair<double, double>> &res,
        int i, int j, double duration = 0, double startTime = 0) const;

    bool hasColisionIntervals(int i, int j) const;

    const boost::icl::interval_set<double>& getNodeColisionIntervals(int i, int j) const;

private:
    std::map<std::pair<int, int>, boost::icl::interval_set<double>> intervals;
};

#endif // CONFLICTAVOIDANCETABLE_H
