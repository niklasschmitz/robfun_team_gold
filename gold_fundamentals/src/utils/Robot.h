#ifndef SRC_ROBOT_H
#define SRC_ROBOT_H

#include "ros/ros.h"
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include "tools.h"
#include "PID.h"
#include "GridPerceptor.h"
#include <queue>


class Robot {
public:
    Robot();

    ~Robot();

    void diffDrive(double left, double right);

    void brake();

    void drive(double distance);

    void turn(double angle);

    void turnRandom();

    void align();

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

    T_POINT2D position;
    T_POINT2D positionGoal;
    double theta;
    double thetaGoal;
    std::queue<T_POINT2D> path;

    PID controller;
    ros::ServiceClient diff_drive;
    GridPerceptor gp;

    create_fundamentals::SensorPacket::ConstPtr sensorData;


    void calculatePosition(const create_fundamentals::SensorPacket::ConstPtr &oldData,
                           const create_fundamentals::SensorPacket::ConstPtr &newData);

    void turnTo(double theta);

    double angleDelta(double theta);

    void driveTo(T_POINT2D position);

    bool reachedGoal();

    void steer();

    bool reachedTheta();

    void spin();

    void executePath();

    bool isCloseTo(T_POINT2D point);

    void followPath(std::queue<T_POINT2D> path);
};

#endif //SRC_ROBOT_H
