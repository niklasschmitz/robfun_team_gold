#include "ros/ros.h"
#include <cmath>
#include <csignal>
#include <cstdlib>
#include "sensor_msgs/LaserScan.h"
#include "create_fundamentals/DiffDrive.h"
#include "tools.h"

const double LASER_OFFSET = 0.12;
const double ROBOT_RADIUS = 0.17425;
const double ROBOT_WHEEL_RADIUS = 0.032;
const double ROBOT_SAFETY_DISTANCE = ROBOT_RADIUS + 0.1;
const double ROBOT_TRACK = 0.258;
const double ROBOT_ANGULAR_SPEED = 10.0; //max: 15.625

ros::ServiceClient diffDrive;


inline double distanceEllipse(double angle) {
    const double a = ROBOT_SAFETY_DISTANCE - LASER_OFFSET;
    const double b = ROBOT_SAFETY_DISTANCE;

    return a * b / sqrt(pow(b * cos(angle), 2.0) + pow(a * sin(angle), 2.0));
}


void brake() {
    ROS_INFO("braking: diffDrive 0 0");
    create_fundamentals::DiffDrive srv;
    srv.request.left = 0.0f;
    srv.request.right = 0.0f;
    diffDrive.call(srv);
}

void turn(double angle) {
    brake();
    
    double distance = angle * ROBOT_TRACK / 2;
    double time = distanceToTime(distance, ROBOT_ANGULAR_SPEED, ROBOT_WHEEL_RADIUS);
    ROS_INFO("turning: diffDrive -%lf %lf angle:%lf time:%lf", ROBOT_ANGULAR_SPEED, ROBOT_ANGULAR_SPEED, angle, time);
    create_fundamentals::DiffDrive srv;
    srv.request.left = -ROBOT_ANGULAR_SPEED * sgn(angle);
    srv.request.right = ROBOT_ANGULAR_SPEED * sgn(angle);
    diffDrive.call(srv);

    ros::Duration(time).sleep();
    brake();
}


void turnRandom() {
    int degree = rand() % 360 - 180;
    double radiant = degree * M_PI / 180.0;
    turn(radiant);
}


void drive() {
    ROS_INFO("driving: diffDrive %lf %lf", ROBOT_ANGULAR_SPEED, ROBOT_ANGULAR_SPEED);
    create_fundamentals::DiffDrive srv;
    srv.request.left = ROBOT_ANGULAR_SPEED;
    srv.request.right = ROBOT_ANGULAR_SPEED;
    diffDrive.call(srv);
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
    ROS_INFO("distance=%f", msg->ranges[msg->ranges.size() / 2]);

    double angle = msg->angle_min;
    bool obstacle = false;

    for (int i = 0; i < msg->ranges.size(); i++) {
        if (msg->ranges[i] < distanceEllipse(i)) {
            obstacle = true;
            break;
        }

        angle += msg->angle_increment;
    }

    if (obstacle) {
        turnRandom();
    } else {
        drive();
    }
}

void mySigintHandler(int sig) {
    ROS_INFO("exiting.. sig:%d", sig);
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

    ros::spin();

    return 0;
}
