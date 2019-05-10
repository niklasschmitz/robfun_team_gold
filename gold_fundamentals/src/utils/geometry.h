
#ifndef SRC_GEOMETRY_H
#define SRC_GEOMETRY_H

#include <cmath>
#include <vector>



struct T_VECTOR2D {
    double x;
    double y;

    // constructor
    T_VECTOR2D(double x = 0, double y = 0)
            : x(x), y(y) {}

    // trivial copy constructor
    T_VECTOR2D &operator=(const T_VECTOR2D &other) = default;

    // equality
    bool operator==(const T_VECTOR2D &other) {
        return (x == other.x && y == other.y);
    }

    // addition
    T_VECTOR2D operator+(const T_VECTOR2D &other) const {
        return T_VECTOR2D(x + other.x, y + other.y);
    }

    // subtraction
    T_VECTOR2D operator-(const T_VECTOR2D &other) const {
        return T_VECTOR2D(x - other.x, y - other.y);
    }

    // dot product
    double operator*(const T_VECTOR2D &other) const {
        return x * other.x + y * other.y;
    }

    // length
    const double length() const {
        return sqrt(x * x + y * y);
    }

    // alias for length()
    const double magnitude() const {
        return length();
    }

    static void normalize(T_VECTOR2D &v) {
        double length = v.length();
        v.x /= length;
        v.y /= length;
    }

    // angle between
    static double angleBetweenVectors(const T_VECTOR2D &v1, const T_VECTOR2D &v2) {
        double cos_between = (v1 * v2) / (v1.length() * v2.length());
        double angle = acos(cos_between);
        return angle;
    }

    // returns angle theta of the complex number interpretation
    const double theta() {
        return fmod(atan2(y, x) + 2.0 * M_PI, 2.0 * M_PI);
    }

    const T_VECTOR2D rotate(double theta) {
        double newX = x * cos(theta) - y * sin(theta);
        double newY = x * sin(theta) + y * cos(theta);
        return T_VECTOR2D(newX, newY);
    }

};

struct T_LINE {
    T_VECTOR2D x0; // support vector
    T_VECTOR2D u; // (normalized) directional vector
};


struct T_RATED_LINE {
    T_LINE line;
    int inliers;
};

double distBetweenLineAndPoint(const T_LINE &line, const T_VECTOR2D &point);

#endif //SRC_GEOMETRY_H
