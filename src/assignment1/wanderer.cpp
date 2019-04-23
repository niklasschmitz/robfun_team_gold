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
const double ROBOT_ANGULAR_SPEED = 10.0; //max: 15.625

ros::ServiceClient diffDrive;
bool obstacle = false; //for testing purposes


inline double distanceFromCenter(double range, double angle) {
    double alpha = angle + M_PI_2;
    double a = sin(alpha) * range;
    double b = cos(alpha) * range;
    return sqrt(pow(a, 2.0) + pow(b + LASER_OFFSET, 2.0));
}


inline double distanceEllipse(double angle) {
    const double a = ROBOT_SAFETY_DISTANCE - LASER_OFFSET;
    const double b = ROBOT_SAFETY_DISTANCE;

    return a * b / sqrt(pow(b * cos(angle), 2.0) + pow(a * sin(angle), 2.0));
}


void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
//    ROS_INFO("angle: min=%f, max=%f", msg->angle_min, msg->angle_max);
//    ROS_INFO("angle_increment= %f", msg->angle_increment);
//    ROS_INFO("time_increment= %f", msg->time_increment);
//    ROS_INFO("scan_time= %f", msg->scan_time);
//    ROS_INFO("range: min=%f, max=%f", msg->range_min, msg->range_max);
    ROS_INFO("distance=%f", msg->ranges[msg->ranges.size() / 2]);

    double angle = msg->angle_min;

    for (int i = 0; i < msg->ranges.size(); i++) {
        if (msg->ranges[i] < distanceEllipse(0)) {
            obstacle = true;
            break;
        }
        else {
            obstacle = false;
        }

        angle += msg->angle_increment;
    }
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


void turn(double angle) {
    brake();

    double distance = angle * ROBOT_TRACK / 2;
    double time = distanceToTime(distance, ROBOT_ANGULAR_SPEED);
    ROS_INFO("turning: diffDrive -10 10 %lf %lf", angle, time);
    create_fundamentals::DiffDrive srv;
    srv.request.left = -ROBOT_ANGULAR_SPEED;
    srv.request.right = ROBOT_ANGULAR_SPEED;
    diffDrive.call(srv);

    ros::Duration(time).sleep();
    brake();
}


void turnRandom() {
    int degree = rand() % 360;
    double radiant = degree * M_PI / 180.0;
    turn(radiant);
}


void drive() {
    ROS_INFO("driving: diffDrive 10 10");
    create_fundamentals::DiffDrive srv;
    srv.request.left = ROBOT_ANGULAR_SPEED;
    srv.request.right = ROBOT_ANGULAR_SPEED;
    diffDrive.call(srv);
}


void mySigintHandler(int sig) {
    ROS_INFO("exiting..");
    brake();

    ros::shutdown();
}


int main(int argc, char **argv) {
    signal(SIGINT, mySigintHandler);
    ros::init(argc, argv, "wanderer", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("scan_filtered", 1, laserCallback);
    diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    signal(SIGINT, mySigintHandler);
    ros::Rate loop_rate(25);

    while (ros::ok()) {
        if (obstacle) {
            turnRandom();
        } else {
            drive();
        }

        ros::spinOnce();

        ROS_INFO("alive");

        loop_rate.sleep();
    }

    mySigintHandler(0);

    return 0;
}
