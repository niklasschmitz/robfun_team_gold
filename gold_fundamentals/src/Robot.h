#ifndef SRC_ROBOT_H
#define SRC_ROBOT_H

#include "ros/ros.h"
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include "tools.h"
#include "PID.h"

class Robot {
public:
    Robot();
    Robot(ros::ServiceClient diff_drive);

    ~Robot();

    void diffDrive(double left, double right);

    static const double ENCODER_STEPS_PER_REVOLUTION;
    static const double LASER_OFFSET;
    static const double MAX_SPEED;
    static const double MIN_SPEED;
    static const double RADIUS;
    static const double SAFETY_DISTANCE;
    static const double TRACK;
    static const double WHEEL_RADIUS;

    double speed;
    PID controller;
    PID controllerLeft;
    PID controllerRight;
    ros::ServiceClient diff_drive;

    create_fundamentals::SensorPacket::ConstPtr sensorData;

//private:

};

#endif //SRC_ROBOT_H
