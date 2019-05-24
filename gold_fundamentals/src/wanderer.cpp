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

void mySigintHandler(int sig) {
    ROS_INFO("exiting.. sig:%d", sig);
    robot->brake();

    ros::shutdown();

    delete(robot);
}


int main(int argc, char **argv) {
    signal(SIGINT, mySigintHandler);
    ros::init(argc, argv, "wanderer", ros::init_options::NoSigintHandler);
    robot = new Robot();
    signal(SIGINT, mySigintHandler);

    robot->wander();

    delete(robot);

    return 0;
}
