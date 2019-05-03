#include "ros/ros.h"
#include <csignal>
#include <cmath>
#include "gold_fundamentals/ExecutePlan.h"
#include "create_fundamentals/DiffDrive.h"
#include "Robot.h"
#include "GridPerceptor.h"


double const MAZE_SIDE_LENGTH = 1.5; // TODO: measure the side length

Robot robot;

void turn(int direction) {
    int amount = direction - robot.direction;

    if (amount = 3)
        amount = -1;
    if (amount = -3)
        amount = 1;

    robot.turn(amount * M_PI_2);
}

bool execute(gold_fundamentals::ExecutePlan::Request &req, gold_fundamentals::ExecutePlan::Response &res) {

    for (int i = 0; i < req.plan.size(); i++) {
        ROS_INFO("%d", req.plan[i]);
        turn(req.plan[i]);
        robot.drive(MAZE_SIDE_LENGTH);
    }

    res.success = true;

    return true;
}

void sensorCallback(const create_fundamentals::SensorPacket::ConstPtr &msg) {
    ROS_INFO("left encoder: %lf, right encoder: %lf", msg->encoderLeft, msg->encoderRight);
    robot.sensorData = msg;
}

void mySigintHandler(int sig) {
    ROS_INFO("exiting.. sig:%d", sig);
    robot.diffDrive(0.0, 0.0);

    ros::shutdown();
}

int main(int argc, char **argv) {
    signal(SIGINT, mySigintHandler);
    ros::init(argc, argv, "plan_executer", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("sensor_packet", 1, sensorCallback);
    ros::ServiceClient diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    ros::ServiceServer service = n.advertiseService("execute_plan", execute);
    signal(SIGINT, mySigintHandler);
    GridPerceptor gp(n);
    robot = Robot(diffDrive, gp);

    ROS_INFO("Ready to execute.");
    ros::spin();

    return 0;
}