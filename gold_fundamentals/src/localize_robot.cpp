#include "ros/ros.h"
#include <cmath>
#include <csignal>
#include <cstdlib>
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include "utils/Robot.h"
#include "utils/tools.h"
#include "utils/GridPerceptor.h"

#include "utils/geometry.h"

Robot* robot;

void mySigintHandler(int sig) {
    ROS_INFO("exiting.. sig:%d", sig);
    robot->brake();

    ros::shutdown();
    delete(robot);
}

void localize() {
    //robot->particleFilter.publishAllParticlesToRviz();
//    robot->particleFilter.publishBestParticleWeight();
    //robot->particleFilter.publishParticleVariance();
//    robot->pf.setUpdateMap();
//    ros::Duration(0.1).sleep();
}


int main(int argc, char **argv) {
    signal(SIGINT, mySigintHandler);
    ros::init(argc, argv, "localize", ros::init_options::NoSigintHandler);
    robot = new Robot();
    signal(SIGINT, mySigintHandler);

    delete(robot);
    return 0;
}

