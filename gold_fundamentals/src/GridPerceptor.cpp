#include "GridPerceptor.h"

GridPerceptor::GridPerceptor() {}

GridPerceptor::GridPerceptor(ros::NodeHandle n) {
    sub_laser = n.subscribe("scan_filtered", 1, &GridPerceptor::laserCallback, this);
}

GridPerceptor::~GridPerceptor() {
}

void GridPerceptor::laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
    ROS_INFO("Hi");
    //ROS_INFO("%i", msg->ranges.size());
    //ROS_INFO("min deg %f", msg->angle_min);
    //ROS_INFO("min deg %f", msg->angle_increment);
    //ROS_INFO("%f", msg->ranges[msg->ranges.size() / 2]);
}

