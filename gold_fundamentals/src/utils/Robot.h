#ifndef SRC_ROBOT_H
#define SRC_ROBOT_H

#include "ros/ros.h"
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include "tools.h"
#include "PID.h"
#include "GridPerceptor.h"
#include <queue>
#include "gold_fundamentals/Pose.h"
#include "gold_fundamentals/Grid.h"
#include "geometry.h"


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

    static const double LASER_OFFSET;
    static const double MAX_SPEED;
    static const double MIN_SPEED;
    static const double RADIUS;
    static const double SAFETY_DISTANCE;
    static const double TRACK;
    static const double WHEEL_RADIUS;

    T_VECTOR2D position;
    double theta;

    PID turnControl;
    PID speedControl;
    PID steerControl;
    PID steerMaxControl;

    ros::ServiceClient diff_drive;
    GridPerceptor gp;

    create_fundamentals::SensorPacket::ConstPtr sensorData;
    ros::Time sensorTime;
    double timeDelta;

    ros::Publisher pose_pub;

    void calculatePosition(const create_fundamentals::SensorPacket::ConstPtr &oldData,
                           const create_fundamentals::SensorPacket::ConstPtr &newData);

    void turnTo(double theta);

    double angleDelta(double theta);

    void driveTo(T_VECTOR2D position);

    bool isCloseTo(T_VECTOR2D point);

    void followPath(std::queue<T_VECTOR2D> path);

    bool reachedGoal(T_VECTOR2D goal);

    void alignToWall();

    void resetPosition();

    void publishPosition();

    void spin(double thetaGoal);

    void steer(std::queue<T_VECTOR2D> path);

    bool reachedTheta(double thetaGoal);
};

#endif //SRC_ROBOT_H
