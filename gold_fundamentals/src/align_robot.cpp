#include "ros/ros.h"
#include <cmath>
#include <csignal>
#include <cstdlib>
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include "utils/Robot.h"
#include "utils/tools.h"
#include "utils/GridPerceptor.h"

Robot* robot;


void mySigintHandler(int sig) {
    ROS_INFO("exiting.. sig:%d", sig);
    robot->brake();

    ros::shutdown();
    delete(robot);
}

void align_robot() {
    robot->align();
}


int main(int argc, char **argv) {
    signal(SIGINT, mySigintHandler);
    ros::init(argc, argv, "align", ros::init_options::NoSigintHandler);
    robot = new Robot();
    signal(SIGINT, mySigintHandler);

    while(ros::ok) {
        align_robot();
        ros::spinOnce();
    }

    delete(robot);
    return 0;
}

