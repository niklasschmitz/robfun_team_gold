#include "ros/ros.h"
#include <cmath>
#include <csignal>
#include <cstdlib>
#include "create_fundamentals/DiffDrive.h"
#include "tools.h"

const double ROBOT_WHEEL_RADIUS = 0.032;
const double ROBOT_TRACK = 0.258;
const double ROBOT_ANGULAR_SPEED = 5.0; //max: 15.625

ros::ServiceClient diffDrive;


void brake() {
    ROS_INFO("braking: diffDrive 0 0");
    create_fundamentals::DiffDrive srv;
    srv.request.left = 0.0f;
    srv.request.right = 0.0f;
    diffDrive.call(srv);
}

// angle in radians
void turn(double angle) {
    brake();

    double distance = angle * ROBOT_TRACK / 2;
    double time = distanceToTime(distance, ROBOT_ANGULAR_SPEED, ROBOT_WHEEL_RADIUS);
    ROS_INFO("turning: diffDrive %lf %lf angle:%lf time:%lf", ROBOT_ANGULAR_SPEED, ROBOT_ANGULAR_SPEED, angle, time);
    create_fundamentals::DiffDrive srv;
    srv.request.left = -ROBOT_ANGULAR_SPEED * sgn(angle);
    srv.request.right = ROBOT_ANGULAR_SPEED * sgn(angle);
    diffDrive.call(srv);

    ros::Duration(time).sleep();
    brake();
}


void drive(double distance) {
    double time = distanceToTime(distance, ROBOT_ANGULAR_SPEED, ROBOT_WHEEL_RADIUS);
    ROS_INFO("driving: diffDrive %lf %lf", ROBOT_ANGULAR_SPEED, ROBOT_ANGULAR_SPEED);
    create_fundamentals::DiffDrive srv;
    srv.request.left = ROBOT_ANGULAR_SPEED;
    srv.request.right = ROBOT_ANGULAR_SPEED;
    diffDrive.call(srv);

    ros::Duration(time).sleep();
    brake();
}


void mySigintHandler(int sig) {
    ROS_INFO("exiting.. sig:%d", sig);
    brake();

    ros::shutdown();
}


// side_length: side length of square in metres
// iterations: number of repetitions
void driveSquare(double side_length, int iterations) {
    // 4 turns make 1 square
    for (int i = 0; i < iterations * 4; ++i) {
        drive(side_length);
        ros::Duration(0.5).sleep();
        turn(M_PI_2);
        ros::Duration(0.5).sleep();
    }
}


int main(int argc, char **argv) {
    signal(SIGINT, mySigintHandler);
    ros::init(argc, argv, "square_no_sensors", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;
    diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    signal(SIGINT, mySigintHandler);

    if (ros::ok()) {
        // drive 5 consecutive squares
        driveSquare(1., 5);
    }

    mySigintHandler(0);

    return 0;
}
