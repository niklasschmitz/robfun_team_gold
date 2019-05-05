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

typedef struct {
    double x;
    double y;
} T_CARTESIAN_COORD;

class GridPerceptor {
public:
    GridPerceptor();

    ~GridPerceptor();

private:
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

    ros::Subscriber sub_laser;

    T_CARTESIAN_COORD convertPolarToCartesian(double theta, double r);
    T_LINE linear_regression(std::vector<T_CARTESIAN_COORD> coordinates);
    std::vector<T_LINE> ransac(std::vector<T_CARTESIAN_COORD> coordinates);
    double distBetweenLineAndPoint(T_LINE line, T_CARTESIAN_COORD point);
    bool testLineSimilarity(std::vector<T_LINE> lines, T_LINE line);
};

#endif //SRC_GP_H
