
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
    T_VECTOR2D &operator=(const T_VECTOR2D &other) {
        x = other.x;
        y = other.y;
    }

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

    // normalizes in place
    void normalize() {
        x /= length();
        y /= length();
    }

    // returns new normalized vector
    static T_VECTOR2D normalize(const T_VECTOR2D &v) {
        T_VECTOR2D normalized_vec;
        double length = v.length();
        normalized_vec.x = v.x / length;
        normalized_vec.y = v.y / length;
        return normalized_vec;
    }

    // angle between
    static double angleBetweenVectors(const T_VECTOR2D &v1, const T_VECTOR2D &v2) {
        double cos_between = (v1 * v2) / (v1.length() * v2.length());
        double angle = acos(cos_between);
        return angle;
    }

    // returns the angle one would have to add to v1 to reach v2
    static double signedAngleBetweenVectors(const T_VECTOR2D &v1, const T_VECTOR2D &v2) {
        return atan2(v1.y, v1.x) - atan2(v2.y, v2.x);
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

    // scalar vector multiplication
    T_VECTOR2D operator*(const double scalar) const {
        return T_VECTOR2D(scalar * x, scalar * y);
    }

//    static const T_VECTOR2D mirror(const T_VECTOR2D &vec_to_mirror, const T_VECTOR2D &mirror_axis) {
//        T_VECTOR2D norm_mirror_axis = normalizeR(mirror_axis);
//        double tmp_x_value = 2 * (vec_to_mirror*norm_mirror_axis) * mirror_axis.x;
//        double tmp_y_value = 2 * (vec_to_mirror*norm_mirror_axis) * mirror_axis.y;
//        T_VECTOR2D tmp_vec = T_VECTOR2D(tmp_x_value, tmp_y_value);
//        return vec_to_mirror - tmp_vec;// * mirror_axis;
//    }

    static const T_VECTOR2D mirror(const T_VECTOR2D &vec_to_mirror, const T_VECTOR2D &mirror_axis) {
        double tmp_val = 2 * (vec_to_mirror*mirror_axis) / (mirror_axis*mirror_axis);

        T_VECTOR2D tmp_vec;
        tmp_vec.x = tmp_val * mirror_axis.x;
        tmp_vec.y = tmp_val * mirror_axis.y;

        return tmp_vec - vec_to_mirror;
    }
};

T_VECTOR2D operator*(const double scalar, const T_VECTOR2D vec);


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
