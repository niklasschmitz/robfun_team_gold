#include "ros/ros.h"
#include <cmath>
#include <csignal>
#include <cstdlib>
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include "utils/Robot.h"
#include "utils/tools.h"
#include "utils/GridPerceptor.h"
#include "gold_fundamentals/ExecutePlan.h"
#include "utils/geometry.h"

Robot* robot;

bool execute(gold_fundamentals::ExecutePlan::Request &req, gold_fundamentals::ExecutePlan::Response &res) {
    if (req.plan.size() == 0) {
        res.success = true;
        return true;
    }

    robot->executePlan(req.plan);

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
    ros::init(argc, argv, "localize", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("execute_plan", execute);
    robot = new Robot();
    signal(SIGINT, mySigintHandler);
    robot->localize();
    robot->playSong(1);
    robot->driveCenterCell();
    ros::spin();
    delete(robot);
    return 0;
}

