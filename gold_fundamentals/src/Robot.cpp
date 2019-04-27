#include "ros/ros.h"
#include "create_fundamentals/DiffDrive.h"

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
    double encoderLeft;
    double encoderRight;
    ros::ServiceClient diff_drive;

//private:

};

const double Robot::ENCODER_STEPS_PER_REVOLUTION = M_PI * 2.0;
const double Robot::LASER_OFFSET = 0.12;
const double Robot::MAX_SPEED = 15.625;
const double Robot::MIN_SPEED = M_PI;
const double Robot::RADIUS = 0.17425;
const double Robot::SAFETY_DISTANCE = RADIUS + 0.1;
const double Robot::TRACK = 0.258;
const double Robot::WHEEL_RADIUS = 0.032;

Robot::Robot() {}

Robot::Robot(ros::ServiceClient diff_drive) :
        diff_drive(diff_drive),
        encoderLeft(0.0),
        encoderRight(0.0)
{
}

void Robot::diffDrive(double left, double right) {
    ROS_INFO("diffDrive %lf %lf", left, right);
    create_fundamentals::DiffDrive srv;
    srv.request.left = left;
    srv.request.right = right;
    diff_drive.call(srv);
}

Robot::~Robot() {}