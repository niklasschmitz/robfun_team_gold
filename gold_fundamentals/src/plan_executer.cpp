#include "ros/ros.h"
#include <csignal>
#include <cmath>
#include "gold_fundamentals/ExecutePlan.h"
#include "create_fundamentals/DiffDrive.h"
#include "Robot.h"
#include "GridPerceptor.h"


double const MAZE_SIDE_LENGTH = 0.8;

Robot *robot;

bool execute(gold_fundamentals::ExecutePlan::Request &req, gold_fundamentals::ExecutePlan::Response &res) {

    for (int i = 0; i < req.plan.size(); i++) {
        ROS_INFO("%d", req.plan[i]);
        robot->turnTo(req.plan[i] * M_PI_2);
        robot->drive(MAZE_SIDE_LENGTH);
    }

    res.success = true;

    return true;
}

void mySigintHandler(int sig) {
    ROS_INFO("exiting.. sig:%d", sig);
    robot->brake();

    delete (robot);

    ros::shutdown();
}

int main(int argc, char **argv) {
    signal(SIGINT, mySigintHandler);
    ros::init(argc, argv, "plan_executer", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("execute_plan", execute);
    signal(SIGINT, mySigintHandler);
    robot = new Robot();
    //TODO: insert align call

    ROS_INFO("Ready to execute.");
    ros::spin();

    delete (robot);

    return 0;
}