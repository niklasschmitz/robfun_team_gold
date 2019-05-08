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
    if (req.plan.size() == 0) {
        res.success = true;
        return true;
    }

    robot->turnTo(req.plan[0] * M_PI_2);

    T_CARTESIAN_COORD dist(MAZE_SIDE_LENGTH, 0);
    T_CARTESIAN_COORD next(0, 0);
    std::queue<T_CARTESIAN_COORD> plan;

    for (int i = 0; i < req.plan.size(); i++) {
        next = next + dist.rotate(req.plan[i]);
        plan.push(next);
    }

    robot->followPath(plan);

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

    ros::spin();

    delete (robot);

    return 0;
}