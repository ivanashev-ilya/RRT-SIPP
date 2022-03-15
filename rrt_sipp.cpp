#include "rrt_sipp.h"

void RRT_SIPP::getNodeVersions(const Map& map, const ConflictAvoidanceTable& CAT,
    const Node &curNode, std::vector<Node>& versions)
{
    struct Event {
        double time;
        bool start;
        int id;

        bool operator<(const Event& other) const {
            return time < other.time || time == other.time && start > other.start;
        }
    };

    std::set<std::pair<int, int>> neigh;
    LineOfSight::findPointNeighborhood(Point(curNode.i, curNode.j), neigh, map);
    std::vector<Event> events;
    int i = 0;
    for (auto& cell : neigh) {
        std::vector<std::pair<double, double>> intervals;
        CAT.getSafeIntervals(intervals, cell.first, cell.second);

        for (auto& interval : intervals) {
            events.push_back(Event{interval.first, true, i});
            events.push_back(Event{interval.second, false, i});
        }
        ++i;
    }

    Node version = curNode;
    std::sort(events.begin(), events.end());
    std::set<int> safeCells;
    double start = 0;
    for (auto event : events) {
        if (event.start) {
            safeCells.insert(event.id);
            start = event.time;
        } else {
            if (safeCells.size() == neigh.size()) {
                version.startTime = start;
                version.endTime = event.time;
                versions.push_back(version);
            }
            safeCells.erase(event.id);
        }
    }
}


double RRT_SIPP::getRequiredWaitingTime(const Map& map, const ConflictAvoidanceTable& CAT,
    const Node& prev, const Node& cur) const
{
    double dist = distance(prev, cur);
    if (prev.g + dist > cur.endTime) {
        return -1;
    }
    double res = std::max(0.0, cur.startTime - prev.g - dist);
    if (prev.g + res > prev.endTime) {
        return -1;
    }

    LineOfSight lineOfSight(prev, cur, map);
    std::vector<std::pair<double, double>> intervals;
    lineOfSight.getCollisionIntervals(intervals, prev.g);
    std::vector<boost::icl::interval_set<double>::iterator> cellIters;
    std::vector<boost::icl::interval_set<double>::iterator> cellEnds;
    boost::icl::interval_set<double> emptySet;

    for (auto& cell : lineOfSight.cells) {
        if (!CAT.hasColisionIntervals(cell.first, cell.second)) {
            cellIters.push_back(emptySet.begin());
            cellEnds.push_back(emptySet.end());
            continue;
        }
        auto& cellIntervals = CAT.getNodeColisionIntervals(cell.first, cell.second);
        cellIters.push_back(cellIntervals.begin());
        cellEnds.push_back(cellIntervals.end());
    }

    while (true) {
        bool resChanged = false;
        for (size_t i = 0; i < cellIters.size(); ++i) {
            for (; cellIters[i] != cellEnds[i] && cellIters[i]->upper() < intervals[i].first + res + CN_EPSILON; ++cellIters[i]) {}
            if (cellIters[i] == cellEnds[i] || cellIters[i]->lower() > intervals[i].second + res - CN_EPSILON) {
                continue;
            }
            if (std::fabs(cellIters[i]->upper() - CN_INFINITY) < CN_EPSILON) {
                return -1;
            }
            res += (cellIters[i]->upper() - intervals[i].first - res);
            if (prev.g + res > prev.endTime) {
                return -1;
            }
            resChanged = true;
        }
        if (!resChanged) {
            break;
        }
    }
    return res;
}
