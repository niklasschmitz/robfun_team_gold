#ifndef SRC_ROBOT_H
#define SRC_ROBOT_H

#include "ros/ros.h"
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include "tools.h"
#include "PID.h"
#include "GridPerceptor.h"

class Robot {
public:
    Robot();
    ~Robot();

    void diffDrive(double left, double right);
    void brake();
    void drive(double distance);
    void turn(double angle);
    void turnRandom();
    void sensorCallback(const create_fundamentals::SensorPacket::ConstPtr &msg);

    ros::Subscriber sub_sensor;

    static const double LOOPRATE;

    static const double ENCODER_STEPS_PER_REVOLUTION;
    static const double LASER_OFFSET;
    static const double MAX_SPEED;
    static const double MIN_SPEED;
    static const double RADIUS;
    static const double SAFETY_DISTANCE;
    static const double TRACK;
    static const double WHEEL_RADIUS;

    double speed;
    int direction;
    PID controller;
    ros::ServiceClient diff_drive;
    GridPerceptor gp;

    create_fundamentals::SensorPacket::ConstPtr sensorData;

//private:

};

#endif //SRC_ROBOT_H
