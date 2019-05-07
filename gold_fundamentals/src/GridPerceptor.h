#ifndef SRC_GP_H
#define SRC_GP_H

#include "sensor_msgs/LaserScan.h"
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

    // assignment
    T_POINT2D &operator=(const T_POINT2D &other) {
        x = other.x;
        y = other.y;
        return *this;
    }

    // equality
    bool operator==(const T_POINT2D& other) {
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
};

typedef struct {
    T_POINT2D x0; // support vector
    T_POINT2D u; // (normalized) directional vector
} T_LINE;

class GridPerceptor {
public:
    GridPerceptor();

    ~GridPerceptor();

private:
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg);

    ros::Subscriber sub_laser;

    T_POINT2D convertPolarToCartesian(double theta, double r);

    T_LINE constructLineParameterForm(T_POINT2D x1, T_POINT2D x2);

    std::vector<T_LINE> ransac(std::vector<T_POINT2D> coordinates);

    double distBetweenLineAndPoint(T_LINE line, T_POINT2D point);

    bool testLineSimilarity(std::vector<T_LINE> lines, T_LINE line);
};

#endif //SRC_GP_H
