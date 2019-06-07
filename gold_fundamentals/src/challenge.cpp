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

Robot* robot = NULL;
std::vector<gold_fundamentals::Pose> *gold = NULL;
std::vector<gold_fundamentals::Pose> *pickup = NULL;


void mySigintHandler(int sig) {
    ROS_INFO("exiting.. sig:%d", sig);
    robot->brake();

    ros::shutdown();
    delete(robot);
}

void goldCallback(const gold_fundamentals::Goal::ConstPtr &msg) {
    if (!gold) {
        gold = new std::vector<gold_fundamentals::Pose>;
        for (gold_fundamentals::Pose p: msg->positions)
            gold->push_back(p);
    }
}

void pickupCallback(const gold_fundamentals::Goal::ConstPtr &msg) {
    if (!pickup) {
        pickup = new std::vector<gold_fundamentals::Pose>;
        for (gold_fundamentals::Pose p: msg->positions)
            pickup->push_back(p);
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
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }

    while (!pickup){
        ROS_INFO("waiting for pickup");
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }

    //TODO: pickup gold and drive to Helipad

    ros::spin();

    delete(robot);
    return 0;
}