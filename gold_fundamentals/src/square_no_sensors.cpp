#include "ros/ros.h"
#include <cmath>
#include <csignal>
#include <cstdlib>
#include "sensor_msgs/LaserScan.h"
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"

const double LASER_OFFSET = 0.12;
const double ROBOT_RADIUS = 0.17425;
const double ROBOT_WHEEL_RADIUS = 0.032;
const double ROBOT_SAFETY_DISTANCE = ROBOT_RADIUS + 0.2;
const double ROBOT_TRACK = 0.235;
const double ROBOT_ANGULAR_SPEED = 5.0; //max: 15.625
const double NUM_SQUARES = 5; // Num of Squares to drive

ros::ServiceClient diffDrive;
bool obstacle = false; //for testing purposes


inline double distanceFromCenter(double range, double angle) {
    double alpha = angle + M_PI_2;
    double a = sin(alpha) * range;
    double b = cos(alpha) * range;
    return sqrt(pow(a, 2.0) + pow(b + LASER_OFFSET, 2.0));
}


void brake() {
    ROS_INFO("braking: diffDrive 0 0");
    create_fundamentals::DiffDrive srv;
    srv.request.left = 0;
    srv.request.right = 0;
    diffDrive.call(srv);
}


double angularToLinear(double angular) {
    return angular * ROBOT_WHEEL_RADIUS;
}


double distanceToTime(double distance, double speed) {
    return std::abs(distance / angularToLinear(speed));
}


int sgn(double val) {
    if (val > 0) { return 1; }
	if (val < 0) { return -1; }
	return 0;
}


// angle in radians
void turn(double angle) {
    brake();

    double distance = angle * ROBOT_TRACK / 2;
    double time = distanceToTime(distance, ROBOT_ANGULAR_SPEED);
    ROS_INFO("turning: diffDrive -10 10 %lf %lf", angle, time);
    create_fundamentals::DiffDrive srv;
    srv.request.left = -ROBOT_ANGULAR_SPEED * sgn(angle);
    srv.request.right = ROBOT_ANGULAR_SPEED * sgn(angle);
    diffDrive.call(srv);

    ros::Duration(time).sleep();
    brake();
}


void drive(double distance) {
    double time = distanceToTime(distance, ROBOT_ANGULAR_SPEED);
    ROS_INFO("driving: diffDrive 10 10");
    create_fundamentals::DiffDrive srv;
    srv.request.left = ROBOT_ANGULAR_SPEED;
    srv.request.right = ROBOT_ANGULAR_SPEED;
    diffDrive.call(srv);

    ros::Duration(time).sleep();
    brake();
}


void mySigintHandler(int sig) {
    ROS_INFO("exiting..");
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
    ros::Rate loop_rate(25);

    for (int i = 0; i < NUM_SQUARES && ros::ok(); i++) {
        driveSquare(1., 1);

        ros::spinOnce();

        ROS_INFO("alive");

        loop_rate.sleep();
    }

    mySigintHandler(0);

    return 0;
}
