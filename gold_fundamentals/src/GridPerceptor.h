#ifndef SRC_GP_H
#define SRC_GP_H

#include "sensor_msgs/LaserScan.h"
#include "ros/ros.h"
#include <cstdlib>

class GridPerceptor {
public:
    GridPerceptor();
    GridPerceptor(ros::NodeHandle n);

    ~GridPerceptor();


private:
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

    ros::Subscriber sub_laser;
};

#endif //SRC_GP_H
