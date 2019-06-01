#include "ros/ros.h"
#include <csignal>
#include <cmath>
#include "gold_fundamentals/ExecutePlan.h"
#include "create_fundamentals/DiffDrive.h"
#include "utils/Robot.h"
#include "utils/GridPerceptor.h"


Robot *robot;

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

    delete (robot);

    ros::shutdown();
}

int main(int argc, char **argv) {
    signal(SIGINT, mySigintHandler);
    ros::init(argc, argv, "smooth_executer", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("execute_plan", execute);
    signal(SIGINT, mySigintHandler);
    robot = new Robot();
    //robot->align();

    ros::spin();

    delete (robot);

    return 0;
}