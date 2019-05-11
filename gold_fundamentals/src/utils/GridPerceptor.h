#ifndef SRC_GP_H
#define SRC_GP_H

#include <cstdlib>
#include <visualization_msgs/Marker.h>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry.h"

double const MAZE_SIDE_LENGTH = 0.8;

class GridPerceptor {
public:
    GridPerceptor();
    ~GridPerceptor();

    std::vector<T_RATED_LINE> getLines();
    T_RATED_LINE getLineWithMostInliers();

private:
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg);

    ros::Publisher marker_pub;
    ros::Subscriber sub_laser;

    std::vector<T_RATED_LINE> lines;

    T_VECTOR2D convertPolarToCartesian(double theta, double r);

    T_LINE constructLineParameterForm(T_VECTOR2D x1, T_VECTOR2D x2);

    std::vector<T_RATED_LINE> ransac(std::vector<T_VECTOR2D> coordinates);

    bool testLineSimilarity(std::vector<T_RATED_LINE> &lines, T_RATED_LINE line);

    void publishLines(std::vector<T_RATED_LINE> lines);
};

#endif //SRC_GP_H
