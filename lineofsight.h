#ifndef LINEOFSIGHT_H
#define LINEOFSIGHT_H
#include "gl_const.h"

#define CN_OBSTACLE 1

#include <vector>
#include <math.h>
#include <node.h>
#include <map.h>
#include <point.h>
#include <algorithm>
#include <tuple>
#include <set>

class LineOfSight
{
public:
    LineOfSight(Node startNode, Node endNode, const Map& map) {
        startPoint = Point(startNode.i, startNode.j);
        endPoint = Point(endNode.i, endNode.j);
        findPointNeighborhood(startPoint, startNeigh, map);
        findPointNeighborhood(endPoint, endNeigh, map);
        cells.insert(startNeigh.begin(), startNeigh.end());
        cells.insert(endNeigh.begin(), endNeigh.end());

        auto [start, end] = getOrderedStartEnd();

        int firstX, lastX, firstY, lastY;
        double coef1, biasDown1, biasUp1;
        double coef2, biasDown2, biasUp2;
        const bool sameX = fabs(start.i - end.i) < CN_EPSILON;
        const bool sameY = fabs(start.j - end.j) < CN_EPSILON;

        if (sameX || sameY) {
            if (sameX) {
                firstX = std::floor(start.i - 0.5 + CN_EPSILON);
                lastX = std::floor(end.i + 0.5 - CN_EPSILON);
                if (start.j < end.j) {
                    firstY = std::floor(start.j + CN_EPSILON);
                    lastY = std::floor(end.j - CN_EPSILON);
                } else {
                    firstY = std::floor(end.j + CN_EPSILON);
                    lastY = std::floor(start.j - CN_EPSILON);
                }
            } else if (sameY) {
                firstX = std::floor(start.i + CN_EPSILON);
                lastX = std::floor(end.i - CN_EPSILON);
                firstY = std::floor(start.j - 0.5 + CN_EPSILON);
                lastY = std::floor(start.j + 0.5 - CN_EPSILON);
            }

            for (int x = firstX; x <= lastX; ++x) {
                for (int y = firstY; y <= lastY; ++y) {
                    cells.insert(std::make_pair(x, y));
                }
            }
        } else {
            Point sideStep = (end - start).dir().ort() * 0.5;
            Point startUp = start + sideStep;
            Point startDown = start - sideStep;
            Point endUp = end + sideStep;
            Point endDown = end - sideStep;

            std::tie(coef1, biasDown1, biasUp1) = getCoefs(startDown, startUp, endDown);
            std::tie(coef2, biasDown2, biasUp2) = end.j < start.j ?
                getCoefs(endDown, startDown, endUp) :
                getCoefs(startUp, endUp, startDown);

            firstX = std::ceil(start.i - 0.5 + CN_EPSILON);
            lastX = std::floor(end.i + 0.5 - CN_EPSILON);
            for (int x = firstX; x <= lastX; ++x) {
                double down1 = coef1 * x + biasDown1;
                double down2 = coef2 * x + biasDown2;
                double up1 = coef1 * x + biasUp1;
                double up2 = coef2 * x + biasUp2;
                firstY = std::ceil(std::max(down1, down2) + CN_EPSILON);
                lastY = std::floor(std::min(up1, up2) - CN_EPSILON);
                for (int y = firstY; y <= lastY; ++y) {
                    for (int dx = -1; dx <= 0; ++dx) {
                        for (int dy = -1; dy <= 0; ++dy) {
                            auto cell = std::make_pair(x + dx, y + dy);
                            cells.insert(cell);
                        }
                    }
                }
            }
        }
    }

    std::pair<Point, Point> getOrderedStartEnd() {
        Point start = startPoint;
        Point end = endPoint;
        if (start.i > end.i) {
            std::swap(start, end);
        }
        return std::make_pair(start, end);
    }

    std::tuple<double, double, double> getCoefs(const Point& startDown, const Point& startUp, const Point& endDown) {
        double coef = (endDown.j - startDown.j) / (endDown.i - startDown.i);
        double biasUp = -startUp.i * coef + startUp.j;
        double biasDown = -startDown.i * coef + startDown.j;
        return std::make_tuple(coef, biasDown, biasUp);
    }

    static void findPointNeighborhood(const Point& point, std::set<std::pair<int, int>>& neigh, const Map& map) {
        int cellX = int(point.i);
        int cellY = int(point.j);
        for (int newX = std::max(0, cellX - 1); newX <= std::min(map.getMapWidth(), cellX + 1); ++newX) {
            for (int newY = std::max(0, cellY - 1); newY <= std::min(map.getMapHeight(), cellY + 1); ++newY) {
                Point closest(point.i, point.j);
                if (newX != cellX) {
                    closest.i = cellX + int(newX > cellX);
                }
                if (newY != cellY) {
                    closest.j = cellY + int(newY > cellY);
                }
                if ((point - closest).length() < 0.5 - CN_EPSILON) {
                    neigh.insert(std::make_pair(newX, newY));
                }
            }
        }
    }

    std::pair<double, double> getPointCollisionInterval(const Point& point, const Point& start, const Point& end) {
        Point bias = end - start;
        Point dir = bias.dir();
        double length = bias.length();
        double dist = std::fabs(bias.j * point.i - bias.i * point.j + end.i * start.j - end.j * start.i) / bias.length();
        if (dist > 0.5) {
            return std::make_pair(0, 0);
        }

        Point biasP = start - point;
        double a = dir.lengthSq();
        double b = 2 * (dir.i * biasP.i + dir.j * biasP.j);
        double c = biasP.lengthSq() - 0.5 * 0.5;

        double vertex = -b / (2 * a);
        double dis = std::sqrt(b * b - 4 * a * c) / (2 * a);
        double r1 = vertex - dis;
        double r2 = vertex + dis;
        double t1 = std::max(0.0, std::min(length, r1));
        double t2 = std::max(0.0, std::min(length, r2));
        return std::make_pair(t1, t2);
    }

    void getCollisionIntervals(std::vector<std::pair<double, double>>& intervals, double startTime = 0) {
        // auto [start, end] = getOrderedStartEnd();

        auto start = startPoint;
        auto end = endPoint;

        double length = (end - start).length();
        double diffX = end.i - start.i;
        double diffY = end.j - start.j;
        for (auto& cell : cells) {
            double coefX = length / diffX;
            double outerXBeg = (double(cell.first) - 0.5 - start.i) * coefX;
            double innerXBeg = outerXBeg + 0.5 * coefX;
            double innerXEnd = outerXBeg + 1.5 * coefX;
            double outerXEnd = outerXBeg + 2 * coefX;
            if (diffX < 0) {
                std::swap(outerXBeg, outerXEnd);
                std::swap(innerXBeg, innerXEnd);
            }

            double coefY = length / diffY;
            double outerYBeg = (double(cell.second) - 0.5 - start.j) * coefY;
            double innerYBeg = outerYBeg + 0.5 * coefY;
            double innerYEnd = outerYBeg + 1.5 * coefY;
            double outerYEnd = outerYBeg + 2 * coefY;
            if (diffY < 0) {
                std::swap(outerYBeg, outerYEnd);
                std::swap(innerYBeg, innerYEnd);
            }

            double beg1 = std::max(outerXBeg, innerYBeg);
            double end1 = std::min(outerXEnd, innerYEnd);
            double beg2 = std::max(outerYBeg, innerXBeg);
            double end2 = std::min(outerYEnd, innerXEnd);

            double beg = CN_INFINITY;
            double end = -CN_INFINITY;
            if (beg1 < end1) {
                beg = std::min(beg, beg1);
                end = std::max(end, end1);
            }
            if (beg2 < end2) {
                beg = std::min(beg, beg2);
                end = std::max(end, end2);
            }

            for (int dx = 0; dx <= 1; ++dx) {
                for (int dy = 0; dy <= 1; ++dy) {
                    auto [pointBeg, pointEnd] = getPointCollisionInterval(
                        Point(cell.first + dx, cell.second + dy), start, end);
                    if (std::fabs(pointEnd - pointBeg) > CN_EPSILON) {
                        beg = std::min(beg, pointBeg);
                        end = std::max(end, pointEnd);
                    }
                }
            }

            beg = std::max(0.0, std::min(length, beg));
            end = std::max(0.0, std::min(length, end));
            intervals.push_back(std::make_pair(beg + startTime, std::max(beg, end) + startTime));
        }
    }

    void getCollisionIntervalsSlow(std::vector<std::pair<double, double>>& intervals) {
        auto [start, end] = getOrderedStartEnd();
        Point bias = end - start;
        double length = bias.length();
        intervals.resize(cells.size(), {CN_INFINITY, -CN_INFINITY});
        for (int i = 0; i <= 10000; ++i) {
            double time = double(i) / 10000.0;

            Point curPoint = start + bias * time;
            int j = 0;
            for (auto& cell : cells) {
                Point closest(
                    std::max(double(cell.first), std::min(double(cell.first + 1), curPoint.i)),
                    std::max(double(cell.second), std::min(double(cell.second + 1), curPoint.j))
                );
                if ((curPoint - closest).length() < 0.5 + CN_EPSILON) {
                    intervals[j].first = std::min(intervals[j].first, time * length);
                    intervals[j].second = std::max(intervals[j].second, time * length);
                }
                ++j;
            }
        }
    }

    /*std::vector<std::pair<int, int>> getCells(int i, int j)
    {
        std::vector<std::pair<int, int>> cells;
        for(int k=0; k<this->cells.size(); k++)
            cells.push_back({i+this->cells[k].first,j+this->cells[k].second});
        return cells;
    }*/
//private:
    std::set<std::pair<int, int>> cells; //cells that are affected by agent's body
    Point startPoint, endPoint;
    std::set<std::pair<int, int>> startNeigh;
    std::set<std::pair<int, int>> endNeigh;
};

#endif // LINEOFSIGHT_H
