#ifndef SRC_GP_H
#define SRC_GP_H

#include <cstdlib>
#include <visualization_msgs/Marker.h>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry.h"

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
