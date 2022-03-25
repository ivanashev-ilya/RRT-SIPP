#include "rrt.h"

RRT::RRT(double Delta) : delta(Delta) {}

RRT::~RRT() {}

double RRT::distance(const Node& lhs, const Node& rhs) {
    return std::sqrt((lhs.i - rhs.i) * (lhs.i - rhs.i) + (lhs.j - rhs.j) * (lhs.j - rhs.j));
}

bool RRT::checkObstacles(const Node& cur, const Node& neigh, const Map& map) const {
    LineOfSight line({cur.i, cur.j}, {neigh.i, neigh.j}, map);
    for (auto& cell : line.cells) {
        if (map.CellIsObstacle(cell.second, cell.first)) {
            return false;
        }
    }
    return true;
}

bool RRT::earlyStop(const Node& cur, const Node& goal, const Map& map) const {
    return distance(cur, goal) <= delta && checkObstacles(cur, goal, map);
}

void RRT::getNodeVersions(const Map& map, const ConflictAvoidanceTable& CAT,
    const Node &curNode, std::vector<Node>& versions)
{
    versions.push_back(curNode);
}

void RRT::addNode(Node& node, const std::list<Node>::iterator& parentIt,
    const Map& map, const ConflictAvoidanceTable& CAT)
{
    double parentDist = distance(*parentIt, node);
    std::vector<Node> versions;
    getNodeVersions(map, CAT, node, versions);
    for (auto& version : versions) {
        for (auto it = parentIt; it != nodes.end() && it->pointId == parentIt->pointId; ++it) {
            double waitingTime = getRequiredWaitingTime(map, CAT, *it, version);
            if (waitingTime != -1) {
                version.g = it->g + parentDist + waitingTime;
                nodes.push_back(version);
                setParent(nodes.back(), &(*it));
            }
        }
    }
}

void RRT::setParent(Node& node, Node* parent) const
{
    node.parent = parent;
}

SearchResult RRT::startSearch(Map &map, const Config &config, const ConflictAvoidanceTable& CAT, std::mt19937 &rng)
{
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    Node cur(map.getStart_j() + 0.5, map.getStart_i() + 0.5, nullptr, 0);
    Node goal(map.getGoal_j() + 0.5, map.getGoal_i() + 0.5);
    cur.pointId = 0;
    std::vector<Node> startVersions;
    getNodeVersions(map, CAT, cur, startVersions);
    nodes.push_back(startVersions[0]);

    int iter = 1;
    sresult.pathfound = false;

    while (!sresult.pathfound) {
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        int elapsedMilliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
        if (elapsedMilliseconds > 1000 * config.maxTime) {
            break;
        }

        cur = Node(map.getRandom_i(rng), map.getRandom_j(rng));
        cur.pointId = iter++;

        double parentDist = CN_INFINITY;
        std::list<Node>::iterator parentIt;
        auto it = nodes.begin();
        while (it != nodes.end()) {
            auto& node = *it;
            double dist = distance(cur, node);
            if (dist < parentDist) {
                parentDist = dist;
                parentIt = it;
            }
            int curPointId = it->pointId;
            for (; it != nodes.end() && it->pointId == curPointId; ++it) {}
        }
        if (parentDist > delta) {
            cur.i = parentIt->i + (cur.i - parentIt->i) * delta / parentDist;
            cur.j = parentIt->j + (cur.j - parentIt->j) * delta / parentDist;
        }
        if (!checkObstacles(cur, *parentIt, map)) {
            continue;
        }

        addNode(cur, parentIt, map, CAT);
    }

    std::vector<Node> goalVersions;
    getNodeVersions(map, CAT, goal, goalVersions);
    Node version = goalVersions.back();
    double bestG = CN_INFINITY;
    Node* parent = nullptr;
    for (auto& node : nodes) {
        if (!checkObstacles(node, goal, map)) {
            continue;
        }

        double waitingTime = getRequiredWaitingTime(map, CAT, node, version);
        if (waitingTime == -1) {
            continue;
        }
        double curG = node.g + waitingTime + distance(node, version);
        if (curG < bestG) {
            bestG = curG;
            parent = &node;
        }
    }
    if (parent != nullptr) {
        sresult.pathfound = true;
        goal = version;
        goal.g = bestG;
        setParent(goal, parent);
    }

    if (sresult.pathfound) {
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        int elapsedMilliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
        sresult.time = static_cast<double>(elapsedMilliseconds) / 1000;
        sresult.pathlength = goal.g;
        makePath(goal);
        sresult.path = &path;
    }

    return sresult;
}

void RRT::makePath(const Node &curNode, const Node* nextNode)
{
    if (nextNode != nullptr) {
        double dist = distance(curNode, *nextNode);
        if (nextNode->g - curNode.g > dist + CN_EPSILON) {
            Node waitNode = curNode;
            waitNode.g = nextNode->g - dist;
            path.push_front(waitNode);
        }
    }
    path.push_front(curNode);
    if (curNode.parent != nullptr) {
        makePath(*(curNode.parent), &curNode);
    }
}
