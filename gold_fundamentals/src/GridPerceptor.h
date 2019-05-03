#ifndef SRC_GP_H
#define SRC_GP_H

#include "sensor_msgs/LaserScan.h"
#include "ros/ros.h"
#include <cstdlib>
#include <vector>

class GridPerceptor {
public:
    GridPerceptor();

    ~GridPerceptor();


private:
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

    ros::Subscriber sub_laser;

    std::vector<double> linear_regression(std::vector<double> x, std::vector<double> y);
};

#endif //SRC_GP_H
