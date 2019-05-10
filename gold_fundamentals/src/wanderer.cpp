#include "ros/ros.h"
#include <cmath>
#include <csignal>
#include <cstdlib>
#include "sensor_msgs/LaserScan.h"
#include "create_fundamentals/DiffDrive.h"
#include "utils/tools.h"
#include "utils/Robot.h"
#include "utils/GridPerceptor.h"

Robot* robot;

inline double distanceEllipse(double angle) {
    const double a = Robot::SAFETY_DISTANCE - Robot::LASER_OFFSET;
    const double b = Robot::SAFETY_DISTANCE;

    return a * b / sqrt(pow(b * cos(angle), 2.0) + pow(a * sin(angle), 2.0));
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
    ROS_INFO("distance=%f", msg->ranges[msg->ranges.size() / 2]);

    double angle = msg->angle_min;
    bool obstacle = false;

    for (int i = 0; i < msg->ranges.size(); i++) {
        if (msg->ranges[i] < distanceEllipse(angle)) {
            obstacle = true;
            break;
        }

        angle += msg->angle_increment;
    }

    if (obstacle) {
        robot->turnRandom();
    } else {
        robot->diffDrive(Robot::MAX_SPEED / 2., Robot::MAX_SPEED / 2.);
    }
}

void mySigintHandler(int sig) {
    ROS_INFO("exiting.. sig:%d", sig);
    robot->brake();

    ros::shutdown();

    delete(robot);
}


int main(int argc, char **argv) {
    signal(SIGINT, mySigintHandler);
    ros::init(argc, argv, "wanderer", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("scan_filtered", 1, laserCallback);
    robot = new Robot();
    signal(SIGINT, mySigintHandler);

    ros::spin();

    delete(robot);

    return 0;
}
