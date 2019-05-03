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


std::vector<double> GridPerceptor::linear_regression(std::vector<double> x, std::vector<double> y) {
    // assume (x, y)'s in cartesian coordinates.
    // regress y given x. model: y = alpha + beta*x

    double x_mean = 0;
    double y_mean = 0;
    for (int i = 0; i < x.size(); ++i) {
        x_mean += x[i];
        y_mean += y[i];
    }
    x_mean /= x.size();
    y_mean /= y.size();

    // compute covariance of x and y and variance of x
    double cov_xy = 0;
    double var_x = 0;
    double std_xi; //tmp variable
    for (int i = 0; i < x.size(); ++i) {
        cov_xy += (x[i] - x_mean) * (y[i] - y_mean);
        std_xi = x[i] - x_mean;
        var_x += std_xi * std_xi;
    }

    // compute alpha and beta
    double beta = cov_xy / var_x;
    double alpha = y_mean - beta * x_mean;

    // TODO return
}
