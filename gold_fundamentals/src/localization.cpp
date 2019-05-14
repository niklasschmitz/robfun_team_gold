#include "ros/ros.h"
#include <csignal>
#include <cmath>
#include "create_fundamentals/DiffDrive.h"
#include "utils/Robot.h"
#include "utils/GridPerceptor.h"


Robot *robot;


void mySigintHandler(int sig) {
    ROS_INFO("exiting.. sig:%d", sig);
    robot->brake();

    delete (robot);

    ros::shutdown();
}

int main(int argc, char **argv) {
    signal(SIGINT, mySigintHandler);
    ros::init(argc, argv, "localization", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;
    signal(SIGINT, mySigintHandler);
    robot = new Robot();
    robot->align();

    delete (robot);

    return 0;
}