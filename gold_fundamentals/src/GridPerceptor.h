#ifndef SRC_GP_H
#define SRC_GP_H

#include "sensor_msgs/LaserScan.h"
#include "ros/ros.h"
#include <cstdlib>
#include <cmath>
#include <vector>

typedef struct {
    double alpha;
    double beta;
} T_LINE;

typedef struct T_POINT2D {
    double x;
    double y;

    T_POINT2D(double x = 0, double y = 0) : x(x), y(y) {}

    T_POINT2D operator-(const T_POINT2D &other) {
        return T_POINT2D(x - other.x, y - other.y);
    }

    T_POINT2D operator+(const T_POINT2D &other) {
        return T_POINT2D(x + other.x, y + other.y);
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

} T_CARTESIAN_COORD;

class GridPerceptor {
public:
    GridPerceptor();

    ~GridPerceptor();

private:
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg);

    ros::Subscriber sub_laser;

    T_CARTESIAN_COORD convertPolarToCartesian(double theta, double r);

    T_LINE linear_regression(std::vector<T_CARTESIAN_COORD> coordinates);

    std::vector<T_LINE> ransac(std::vector<T_CARTESIAN_COORD> coordinates);

    double distBetweenLineAndPoint(T_LINE line, T_CARTESIAN_COORD point);

    bool testLineSimilarity(std::vector<T_LINE> lines, T_LINE line);
};

#endif //SRC_GP_H
