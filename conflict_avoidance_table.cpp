#include "conflict_avoidance_table.h"


bool ConflictAvoidanceTable::hasColisionIntervals(int i, int j) const {
    auto pair = std::make_pair(i, j);
    return intervals.find(pair) != intervals.end();
}


const boost::icl::interval_set<double>&ConflictAvoidanceTable::getNodeColisionIntervals(int i, int j) const {
    return intervals.at(std::make_pair(i, j));
}


void ConflictAvoidanceTable::addInterval(int i, int j, double start, double end) {
    auto pair = std::make_pair(i, j);
    if (intervals.find(pair) == intervals.end()) {
        intervals[pair] = boost::icl::interval_set<double>();
    }
    boost::icl::interval<double>::type iclInterval(start, end);
    intervals[pair].insert(iclInterval);
}

void ConflictAvoidanceTable::addObstaclePath(const std::vector<Node>& nodes, const Map& map)
{
    double curSectionStart = 0;
    std::map<std::pair<int, int>, double> lastNodesStart;
    for (size_t i = 0; i < nodes.size() - 1; ++i) {
        LineOfSight lineOfSight(nodes[i], nodes[i + 1], map);
        std::vector<std::pair<double, double>> intervals;
        lineOfSight.getCollisionIntervals(intervals, curSectionStart);
        std::map<std::pair<int, int>, double> newLastNodesStart;
        int j = 0;
        for (auto& cell : lineOfSight.cells) {
            auto interval = intervals[j];
            if (lastNodesStart.find(cell) != lastNodesStart.end()) {
                interval.first = lastNodesStart[cell];
            }
            if (lineOfSight.endNeigh.find(cell) != lineOfSight.endNeigh.end()) {
                if (i == nodes.size() - 2) {
                    interval.second = CN_INFINITY;
                } else {
                    newLastNodesStart[cell] = intervals[j++].first;
                    continue;
                }
            }
            addInterval(cell.first, cell.second, interval.first, interval.second);
            ++j;
        }
        curSectionStart += (lineOfSight.endPoint - lineOfSight.startPoint).length();
        lastNodesStart = newLastNodesStart;
    }
}

void ConflictAvoidanceTable::getSafeIntervals(std::vector<std::pair<double, double>> &res,
    int i, int j, double duration, double startTime) const
{
    auto pair = std::make_pair(i, j);
    if (intervals.find(pair) == intervals.end()) {
        res.push_back(std::make_pair(startTime, CN_INFINITY));
        return;
    }

    auto it = intervals.at(pair).begin();
    auto end = intervals.at(pair).end();
    double prevUpper = startTime;
    for (it; it != end; ++it) {
        const boost::icl::interval<double>::type& interval = *it;
        if (interval.lower() - prevUpper > duration + CN_EPSILON) {
            res.push_back(std::make_pair(prevUpper, interval.lower()));
        }
        prevUpper = interval.upper();
    }
    res.push_back(std::make_pair(prevUpper, CN_INFINITY));
}
