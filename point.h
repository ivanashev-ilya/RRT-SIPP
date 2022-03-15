#ifndef POINT_H
#define POINT_H

#include <math.h>

struct Point
{
    double i, j;

    Point(double x = 0, double y = 0) {
        i = x;
        j = y;
    }

    Point operator+(const Point& other) const {
        return Point(i + other.i, j + other.j);
    }

    Point operator-(const Point& other) const {
        return Point(i - other.i, j - other.j);
    }

    Point operator*(double mul) const {
        return Point(i * mul, j * mul);
    }

    Point operator/(double div) const {
        return Point(i / div, j / div);
    }

    double lengthSq() const {
        return i * i + j * j;
    }

    double length() const {
        return std::sqrt(i * i + j * j);
    }

    Point ort() const {
        return Point(-j, i);
    }

    Point dir() const {
        return *this / length();
    }
};

#endif // POINT_H
