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
    exit(1);
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

//int driveTo(T_VECTOR2D point) {
//    DiscreteLocalizer localizer;
//
//    // wait until map is received
//    while (!localizer.received_map) {
//        ros::Duration(0.5).sleep();
//        ros::spinOnce();
//    }
//
//    Maze* maze = (localizer.maze);
//    PathPlanner planner;
//
//    Cell* start = maze->getCell(robot->getCellxy());
//    Cell* goal = maze->getCell(point.x, point.y);
//
//    // calculate path
//    planner.populateBreadthFirstSearch(*maze, goal);
//    std::vector<int> plan = planner.extractPlan(start, goal);
//
//    // follow path
//    robot->executePlan(plan);
//
//    bool arrived = robot->getCellxy() == point;
//    return arrived;
//}

int drivePlan(std::vector<int> plan, T_VECTOR2D goal_point) {
    // follow path
    robot->executePlan(plan);

    bool arrived = robot->getCellxy() == goal_point;
    return arrived;
}

std::vector<int> getPlanToPoint(T_VECTOR2D point) {
    DiscreteLocalizer localizer;

    // wait until map is received
    while (!localizer.received_map) {
        ros::Duration(0.1).sleep();
        ros::spinOnce();
    }

    Maze* maze = (localizer.maze);
    PathPlanner planner;

    Cell* start = maze->getCell(robot->getCellxy());
    Cell* goal = maze->getCell(point.x, point.y);

    // calculate path
    planner.populateBreadthFirstSearch(*maze, goal);
    std::vector<int> plan = planner.extractPlan(start, goal);

    return plan;
}

struct BestPlan {
    std::vector<int> bestPlan;
    int index;

    BestPlan(std::vector<int> bestPlan, int index) : bestPlan(bestPlan), index(index) {}
};

BestPlan getBestPlan(std::vector<std::vector<int > > possible_plans) {
    std::vector<int> best_plan;
    int best_plan_length = 1000;
    int best_plan_index = 500;
    for(int plan_idx=0; plan_idx < possible_plans.size(); plan_idx++) {
        int plan_length = possible_plans[plan_idx].size();
        if(plan_length < best_plan_length) {
            best_plan = possible_plans[plan_idx];
            best_plan_length = plan_length;
            best_plan_index = plan_idx;
        }
    }

    return BestPlan(best_plan, best_plan_index);
}

void traverse_positions(std::vector<T_VECTOR2D>* positions) {
    while(positions->size() > 0) {
        //collect plans to all remaining positions
        std::vector<std::vector<int> > possible_plans;
        for(T_VECTOR2D g: *positions) {
            possible_plans.push_back(getPlanToPoint(g));
        }

        BestPlan bestPlan = getBestPlan(possible_plans);

        // remove the position we will drive to now and execute the plan
        if(!drivePlan(bestPlan.bestPlan, positions->at(bestPlan.index))) {
           continue;
        } else {
            robot->digForGold();
            positions->erase(positions->begin() + bestPlan.index);
        }
    }
}

void driveToShortestGoal(std::vector<T_VECTOR2D>* positions) {
    bool arrived = false;
    while(!arrived) {
        //collect plans to all remaining positions
        std::vector<std::vector<int> > possible_plans;
        for(T_VECTOR2D g: *positions) {
            possible_plans.push_back(getPlanToPoint(g));
        }

        BestPlan bestPlan = getBestPlan(possible_plans);

        // remove the position we will drive to now and execute the plan
        if(!drivePlan(bestPlan.bestPlan, positions->at(bestPlan.index))) {
            continue;
        } else {
            arrived = true;
        }
    }
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
        ros::Duration(0.1).sleep();
        ros::spinOnce();
    }

    while (!pickup){
        ROS_INFO("waiting for pickup");
        ros::Duration(0.1).sleep();
        ros::spinOnce();
    }

    traverse_positions(gold);
    driveToShortestGoal(pickup);

    robot->playSong(1);
//    mySigintHandler(0);

    ROS_INFO("We got %d gold.", robot->gold_count);

    ros::shutdown();
    delete(robot);

    return 0;
}