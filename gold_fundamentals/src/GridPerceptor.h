#ifndef SRC_GP_H
#define SRC_GP_H

#include "sensor_msgs/LaserScan.h"
#include "ros/ros.h"
#include <cstdlib>
#include <cmath>
#include <vector>


typedef struct {
    double x;
    double y;
} T_POINT2D;

typedef struct {
    T_POINT2D x0; // support vector
    T_POINT2D u; // (normalized) directional vector
} T_LINE;

class GridPerceptor {
public:
    GridPerceptor();

    ~GridPerceptor();

private:
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

    ros::Subscriber sub_laser;

    T_POINT2D convertPolarToCartesian(double theta, double r);
    T_LINE constructLineParameterForm(T_POINT2D x1, T_POINT2D x2);
    std::vector<T_LINE> ransac(std::vector<T_POINT2D> coordinates);
    double distBetweenLineAndPoint(T_LINE line, T_POINT2D point);
    bool testLineSimilarity(std::vector<T_LINE> lines, T_LINE line);
};

#endif //SRC_GP_H
