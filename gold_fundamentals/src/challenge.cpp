#include "ros/ros.h"
#include <cmath>
#include <csignal>
#include <cstdlib>
#include "utils/Robot.h"
#include "utils/tools.h"
#include "utils/geometry.h"
#include "gold_fundamentals/MoveToPosition.h"
#include "gold_fundamentals/Goal.h"
#include "gold_fundamentals/Pose.h"
#include <DiscreteLocalizer.h>
#include <PathPlanner.h>

Robot* robot = NULL;
std::vector<T_VECTOR2D> *gold = NULL;
std::vector<T_VECTOR2D> *pickup = NULL;


void mySigintHandler(int sig) {
    ROS_INFO("exiting.. sig:%d", sig);
    robot->brake();

    ros::shutdown();
    delete(robot);
    exit 1;
}

void goldCallback(const gold_fundamentals::Goal::ConstPtr &msg) {
    if (!gold) {
        gold = new std::vector<T_VECTOR2D>;
        for (gold_fundamentals::Pose p: msg->positions)
            gold->push_back(T_VECTOR2D(p.row, p.column));
    }
}

void pickupCallback(const gold_fundamentals::Goal::ConstPtr &msg) {
    if (!pickup) {
        pickup = new std::vector<T_VECTOR2D>;
        for (gold_fundamentals::Pose p: msg->positions)
            pickup->push_back(T_VECTOR2D(p.row, p.column));
    }
}

int driveTo(T_VECTOR2D point) {
    DiscreteLocalizer localizer;

    // wait until map is received
    while (!localizer.received_map) {
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }

    Maze* maze = (localizer.maze);
    PathPlanner planner;

    Cell* start = maze->getCell(robot->getCellxy());
    Cell* goal = maze->getCell(point.x, point.y);

    // calculate path
    planner.populateBreadthFirstSearch(*maze, goal);
    std::vector<int> plan = planner.extractPlan(start, goal);

    // follow path
    robot->executePlan(plan);

    bool arrived = robot->getCellxy() == point;
    return arrived;
}

int main(int argc, char **argv) {
    signal(SIGINT, mySigintHandler);
    ros::init(argc, argv, "planning", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;
    ros::Subscriber sub_gold = n.subscribe("gold", 1, goldCallback);
    ros::Subscriber sub_pickup = n.subscribe("pickup", 1, pickupCallback);

    robot = new Robot();

    signal(SIGINT, mySigintHandler);

    robot->localize();

    while (!gold){
        ROS_INFO("waiting for gold");
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }

    while (!pickup){
        ROS_INFO("waiting for pickup");
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }

    //pickup gold and drive to Helipad
    for (T_VECTOR2D g: *gold) {
        while(!driveTo(g)) {
            ROS_INFO("Retry Gold (%lf/%lf)", g.x, g.y);
        }
        robot->digForGold();
        ROS_INFO("gold count %d", robot->gold_count);
    }

    for (T_VECTOR2D p: *pickup) {
        while(!driveTo(p)) {

        }
        //TODO pickup
        robot->playSong(1);
        mySigintHandler(0);
    }

    ROS_INFO("We got %d gold.", robot->gold_count);

    mySigintHandler(0);

    return 0;
}