#include "ros/ros.h"
#include <cmath>
#include <csignal>
#include <cstdlib>
#include "create_fundamentals/DiffDrive.h"
#include "tools.h"
#include "Robot.h"
#include "GridPerceptor.h"

const double ROBOT_WHEEL_RADIUS = 0.032;
const double ROBOT_TRACK = 0.258;
const double ROBOT_ANGULAR_SPEED = 5.0; //max: 15.625

ros::ServiceClient diffDrive;

Robot robot;

void mySigintHandler(int sig) {
    ROS_INFO("exiting.. sig:%d", sig);
    robot.brake();

    ros::shutdown();
}


// side_length: side length of square in metres
// iterations: number of repetitions
void driveSquare(double side_length, int iterations) {
    // 4 turns make 1 square
    for (int i = 0; i < iterations * 4; ++i) {
        robot.drive(side_length);
        ros::Duration(0.5).sleep();
        robot.turn(M_PI_2);
        ros::Duration(0.5).sleep();
    }
}


int main(int argc, char **argv) {
    signal(SIGINT, mySigintHandler);
    ros::init(argc, argv, "square_no_sensors", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;
    diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    signal(SIGINT, mySigintHandler);
    GridPerceptor gp(n);
    robot = Robot(diffDrive, gp);
    if (ros::ok()) {
        // drive 5 consecutive squares
        driveSquare(1., 5);
    }

    mySigintHandler(0);

    return 0;
}
