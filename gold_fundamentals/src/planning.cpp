#include "ros/ros.h"
#include <cmath>
#include <csignal>
#include <cstdlib>
#include "utils/Robot.h"
#include "utils/tools.h"
#include "utils/geometry.h"
#include "gold_fundamentals/MoveToPosition.h"

Robot* robot;

bool execute(gold_fundamentals::MoveToPosition::Request &req, gold_fundamentals::MoveToPosition::Response &res) {
    //TODO: drive robot to (x,y)

    res.success = true;
    return true;
}


void mySigintHandler(int sig) {
    ROS_INFO("exiting.. sig:%d", sig);
    robot->brake();

    ros::shutdown();
    delete(robot);
}


int main(int argc, char **argv) {
    signal(SIGINT, mySigintHandler);
    ros::init(argc, argv, "planning", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("move_to_position", execute);
    robot = new Robot();
    signal(SIGINT, mySigintHandler);
    robot->localize();
    ros::spin();
    delete(robot);
    return 0;
}