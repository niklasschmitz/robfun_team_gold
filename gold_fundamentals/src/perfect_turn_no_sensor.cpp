#include "ros/ros.h"
#include <cmath>
#include <csignal>
#include <cstdlib>
#include "create_fundamentals/DiffDrive.h"
#include "tools.h"

const double ROBOT_WHEEL_RADIUS = 0.032;
const double ROBOT_TRACK = 0.258;
const double ROBOT_ANGULAR_SPEED = 2 * M_PI; //max: 15.625

ros::ServiceClient diffDrive;

void brake() {
    ROS_INFO("braking: diffDrive 0 0");
    create_fundamentals::DiffDrive srv;
    srv.request.left = 0;
    srv.request.right = 0;
    diffDrive.call(srv);
}


// angle in radians
void turn(double angle) {
    double distance = angle * ROBOT_TRACK / 2;
    double time = distanceToTime(distance, ROBOT_ANGULAR_SPEED, ROBOT_WHEEL_RADIUS);
    ROS_INFO("turning: diffDrive -10 10 %lf %lf", angle, time);
    create_fundamentals::DiffDrive srv;
    srv.request.left = -ROBOT_ANGULAR_SPEED * sgn(angle);
    srv.request.right = ROBOT_ANGULAR_SPEED * sgn(angle);
    diffDrive.call(srv);

    ros::Duration(time).sleep();
    brake();
}


void mySigintHandler(int sig) {
    ROS_INFO("exiting..");
    brake();

    ros::shutdown();
}


int main(int argc, char **argv) {
    signal(SIGINT, mySigintHandler);
    ros::init(argc, argv, "perfect_turn_no_sensor", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;
    diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");

    turn(10 * M_PI);

    mySigintHandler(0);

    return 0;
}
