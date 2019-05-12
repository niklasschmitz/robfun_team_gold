
#ifndef SRC_GEOMETRY_H
#define SRC_GEOMETRY_H

#include <cmath>
#include <vector>

#include "tools.h"
#include "ros/ros.h"


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

    // add and assignment
    void operator+=(const T_VECTOR2D &other) {
        x = x + other.x;
        y = y + other.y;
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
        double len = length();
        x /= len;
        y /= len;
    }

    // returns new normalized vector
    static T_VECTOR2D normalize(const T_VECTOR2D &v) {
        T_VECTOR2D normalized_vec;
        double length = v.length();
        normalized_vec.x = v.x / length;
        normalized_vec.y = v.y / length;
        return normalized_vec;
    }

    const bool isvalid() const {
        if(isnan(x) || isnan(y)) {
            return false;
        }
        return true;
    }

    // angle between
    static double angleBetweenVectors(const T_VECTOR2D &v1, const T_VECTOR2D &v2) {
        double cos_between = (v1 * v2) / (v1.length() * v2.length());
        double angle = acos(cos_between);
        return angle;
    }

    // calculates the angle between the robot (0,1) and any direction vector
    static double angleBetweenRobotAndVector(const T_VECTOR2D &vec) {
        T_VECTOR2D v1 = T_VECTOR2D(0,1);
        //T_VECTOR2D vec_mapped = map_to_positive_y_space(vec.rotate(-M_PI_2));

        //ROS_INFO("vec x %lf, y %lf", vec_mapped.x, vec_mapped.y);
//        ROS_INFO("vec x %lf, y %lf", vec.x, vec.y);
        //ROS_INFO("vec theta %lf", vec.theta());
        //T_VECTOR2D new_vec;
        //new_vec = vec.rotate(M_PI_2);
        double angle = vec.theta() ;

        if(angle > M_PI) {
            angle = angle - M_PI;
        }

        return angle;



//        double angle = angleBetweenVectors(v1, vec);
//        if(angle == M_PI_2) {
//            return M_PI_2;
//        }
//
//        double signed_angle = (-1) * fmod(angleBetweenVectors(v1,vec), M_PI_2) * sgn(vec_mapped.x);
        //return signed_angle;
    }

    // maps a 360° space to space where y>=0, if y=0 x will be > 0
    static T_VECTOR2D map_to_positive_y_space(const T_VECTOR2D &vec) {
       if(vec.y < 0) {
           return T_VECTOR2D(vec.x*(-1), vec.y*(-1));
       }  else if (vec.y == 0) {
           // handles edge case y=0, maps x to space >0
           if(vec.x < 0) {
               return T_VECTOR2D(vec.x*(-1), vec.y);
           }
       }

       return vec;
    }

    // returns angle theta of the complex number interpretation
    const double theta() const {
        return fmod(atan2(y, x) + 2.0 * M_PI, 2.0 * M_PI);
    }

    const T_VECTOR2D rotate(double theta) const {
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


struct T_MATRIX2D {
    T_VECTOR2D column1_vec;
    T_VECTOR2D column2_vec;

    T_MATRIX2D(T_VECTOR2D column1_vec, T_VECTOR2D column2_vec)
            : column1_vec(column1_vec), column2_vec(column2_vec) {}

    const double determinant() {
        return column1_vec.x * column2_vec.y - column1_vec.y * column2_vec.x;
    }

    const bool isvalid() const {
        if(column1_vec.isvalid() && column2_vec.isvalid()) {
            return true;
        }
        return false;
    }
};

struct T_LINE {
    T_VECTOR2D x0; // support vector
    T_VECTOR2D u; // (normalized) directional vector

    T_LINE()
    : x0(T_VECTOR2D(NAN,NAN)), u(T_VECTOR2D(NAN,NAN)) {}

    T_LINE(T_VECTOR2D x0, T_VECTOR2D u)
    : x0(x0), u(u) {}

    const bool isvalid() const {
        if(x0.isvalid() && u.isvalid()) {
            return true;
        }
        return false;
    }
};


struct T_RATED_LINE {
    T_LINE line;
    int inliers;

    T_RATED_LINE()
    : line(T_VECTOR2D(NAN,NAN), T_VECTOR2D(NAN,NAN)), inliers(0) {}

    T_RATED_LINE(T_LINE line, int inliers)
    : line(line), inliers(inliers) {}

    const bool isvalid() const {
        if(line.isvalid() && inliers > 0) {
            return true;
        }
        return false;
    }

};

double distBetweenLineAndPoint(const T_LINE &line, const T_VECTOR2D &point);
T_VECTOR2D intersectionOfTwoLines(const T_LINE &line1, const T_LINE &line2);


#endif //SRC_GEOMETRY_H
