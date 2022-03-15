#include "rrt_star_sipp.h"

void RRT_STAR_SIPP::updateSubtree(Node* node, const Map& map, const ConflictAvoidanceTable& CAT) {
    for (auto successor : node->successors) {
        double dist = distance(*node, *successor);
        double waitingTime = getRequiredWaitingTime(map, CAT, *node, *successor);
        assert(waitingTime >= 0);
        double newG = node->g + dist + waitingTime;
        if (newG < successor->g) {
            successor->g = newG;
            updateSubtree(successor, map, CAT);
        }
    }
}

void RRT_STAR_SIPP::addNode(Node& node, const std::list<Node>::iterator& parentIt,
    const Map& map, const ConflictAvoidanceTable& CAT)
{
    int n = nodes.size() + 1;
    double maxDist = delta; //std::min(2 * std::pow(std::log(n) / n, 1.0 / 3), delta);
    std::vector<Node> versions;
    getNodeVersions(map, CAT, node, versions);
    for (auto& version : versions) {
        double bestG = CN_INFINITY;
        Node* parent = nullptr;
        std::vector<Node*> neighbors;
        for (auto& neigh : nodes) {
            if (!checkObstacles(neigh, version, map)) {
                continue;
            }
            double dist = distance(version, neigh);
            if (dist <= maxDist + CN_EPSILON) {
                neighbors.push_back(&neigh);
                double waitingTime = getRequiredWaitingTime(map, CAT, neigh, version);
                if (waitingTime == -1) {
                    continue;
                }
                double curG = neigh.g + dist + waitingTime;
                if (curG < bestG) {
                    parent = &neigh;
                    bestG = curG;
                }
            }
        }
        if (parent == nullptr) {
            continue;
        }

        version.g = bestG;
        nodes.push_back(version);
        setParent(nodes.back(), parent);
        std::vector<Node*> updatedNeighbors;

        for (auto neigh : neighbors) {
            if (!checkObstacles(version, *neigh, map)) {
                continue;
            }
            double waitingTime = getRequiredWaitingTime(map, CAT, version, *neigh);
            if (waitingTime == -1) {
                continue;
            }
            double newG = version.g + distance(version, *neigh) + waitingTime;
            if (newG < neigh->g) {
                neigh->g = newG;
                setParent(*neigh, &nodes.back());
                updatedNeighbors.push_back(neigh);
            }
        }

        for (auto neigh : updatedNeighbors) {
            updateSubtree(neigh, map, CAT);
        }
    }
}

void RRT_STAR_SIPP::setParent(Node& node, Node* parent) const
{
    if (node.parent != nullptr) {
        node.parent->successors.erase(&node);
    }
    node.parent = parent;
    parent->successors.insert(&node);
}
