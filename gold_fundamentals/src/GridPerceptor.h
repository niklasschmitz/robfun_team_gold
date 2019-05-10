#ifndef SRC_GP_H
#define SRC_GP_H

#include "sensor_msgs/LaserScan.h"
#include <visualization_msgs/Marker.h>
#include "ros/ros.h"
#include <cstdlib>
#include <cmath>
#include <vector>


struct T_POINT2D {
    double x;
    double y;

    // constructor
    T_POINT2D(double x = 0, double y = 0)
            : x(x), y(y) {}

    // trivial copy constructor
    T_POINT2D &operator=(const T_POINT2D &other) = default;

    // equality
    bool operator==(const T_POINT2D &other) {
        return (x == other.x && y == other.y);
    }

    // addition
    T_POINT2D operator+(const T_POINT2D &other) const {
        return T_POINT2D(x + other.x, y + other.y);
    }

    // subtraction
    T_POINT2D operator-(const T_POINT2D &other) const {
        return T_POINT2D(x - other.x, y - other.y);
    }

    // dot product
    double operator*(const T_POINT2D &other) const {
        return x * other.x + y * other.y;
    }

    // length
    static double getLength(const T_POINT2D &v) {
        return sqrt(v.x * v.x + v.y * v.y);
    }

    static void normalize(T_POINT2D &v) {
        double length = getLength(v);
        v.x /= length;
        v.y /= length;
    }

    // angle between
    static double angleBetweenVectors(const T_POINT2D &v1, const T_POINT2D &v2) {
        double cos_between = (v1 * v2) / (getLength(v1) * getLength(v2));
        double angle = std::acos(cos_between);
        return angle;
    }
    
    const double magnitude() {
        return sqrt(pow(x, 2) + pow(y, 2));
    }

    const double theta() {
        return fmod(atan2(y, x) + 2.0 * M_PI, 2.0 * M_PI);
    }

    const T_POINT2D rotate(double theta) {
        double newX = x * cos(theta) - y * sin(theta);
        double newY = x * sin(theta) + y * cos(theta);
        return T_POINT2D(newX, newY);
    }
};

struct T_LINE {
    T_POINT2D x0; // support vector
    T_POINT2D u; // (normalized) directional vector
};

struct T_RATED_LINE {
    T_LINE line;
    int inliers;
};


class GridPerceptor {
public:
    GridPerceptor();
    ~GridPerceptor();

private:
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg);

    ros::Publisher marker_pub;
    ros::Subscriber sub_laser;

    T_POINT2D convertPolarToCartesian(double theta, double r);

    T_LINE constructLineParameterForm(T_POINT2D x1, T_POINT2D x2);

    std::vector<T_RATED_LINE> ransac(std::vector<T_POINT2D> coordinates);

    double distBetweenLineAndPoint(T_LINE line, T_POINT2D point);

    bool testLineSimilarity(std::vector<T_RATED_LINE> &lines, T_RATED_LINE line);

    void publishLines(std::vector<T_RATED_LINE> lines);
};

#endif //SRC_GP_H
