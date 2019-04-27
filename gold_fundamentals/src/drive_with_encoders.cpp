#include "ros/ros.h"
#include <cmath>
#include <csignal>
#include <cstdlib>
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include "Robot.cpp"
#include "PID.cpp"
#include "tools.h"

Robot robot;

void sensorCallback(const create_fundamentals::SensorPacket::ConstPtr &msg) {
    ROS_INFO("left encoder: %lf, right encoder: %lf", msg->encoderLeft, msg->encoderRight);
    robot.sensorData = msg;
}

void mySigintHandler(int sig) {
    ROS_INFO("exiting.. sig:%d", sig);
    robot.diffDrive(0.0, 0.0);

    ros::shutdown();
}


void drive(double distance) {
    const int RATE = 100;

    while (robot.sensorData == NULL){
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }

    PID pid = PID(1.0 / RATE, Robot::MAX_SPEED, -Robot::MAX_SPEED, 1.0, 0.0, 0.0);

    double setpoint = robot.sensorData->encoderLeft + distance * Robot::ENCODER_STEPS_PER_REVOLUTION / (M_PI * 2.0 * Robot::WHEEL_RADIUS);
    double pv = robot.sensorData->encoderLeft;
    double prev_encoder = robot.sensorData->encoderLeft;
    double out = 1.0;

    ros::Rate loop_rate(RATE);
    while (ros::ok() && abs(setpoint - pv) > 0.05) {
        ros::spinOnce();

        double diff = abs(robot.encoderLeft - prev_encoder) * sgn(out);
        pv += diff;

        out = pid.calculate(setpoint, pv);
        robot.diffDrive(out, out);
        loop_rate.sleep();
    }
}


int main(int argc, char **argv) {
    signal(SIGINT, mySigintHandler);
    ros::init(argc, argv, "square_with_encoders", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("sensor_packet", 1, sensorCallback);
    ros::ServiceClient diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    signal(SIGINT, mySigintHandler);
    robot = Robot(diffDrive);

    drive(1.0);

    return 0;
}
