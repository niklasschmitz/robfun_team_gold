#include "ros/ros.h"
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <DiscreteLocalizer.h>
#include <PathPlanner.h>
#include "utils/Robot.h"
#include "utils/tools.h"
#include "utils/geometry.h"
#include "gold_fundamentals/MoveToPosition.h"

Robot* robot;

bool execute(gold_fundamentals::MoveToPosition::Request &req, gold_fundamentals::MoveToPosition::Response &res) {
    //TODO: drive robot to (x,y)

    DiscreteLocalizer *localizer = new DiscreteLocalizer();

    // wait until map is received
    while (!localizer->received_map) {
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }

    Maze* maze = (localizer->maze);
    PathPlanner planner;

    // TODO: replace start with robot pose
    Cell* start = maze->getCell(0,0);
    Cell* goal = maze->getCell(req.row, req.column);

    // calculate path
    planner.populateBreadthFirstSearch(*maze, goal);
    std::vector<int> plan = planner.extractPlan(start, goal);

    // follow path
    robot->executePlan(plan);

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