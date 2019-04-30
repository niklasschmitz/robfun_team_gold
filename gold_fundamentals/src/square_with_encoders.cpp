#include "ros/ros.h"
#include <cmath>
#include <csignal>
#include <cstdlib>
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include "Robot.h"
#include "tools.h"
#include "GridPerceptor.h"

ros::ServiceClient diffDrive;

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

// side_length: side length of square in metres
// iterations: number of repetitions
void driveSquare(double side_length, int iterations) {
    // 4 turns make 1 square
    for (int i = 0; i < iterations * 4; ++i) {
        robot.drive(side_length);
        robot.turn(M_PI_2);
    }
}

int main(int argc, char **argv) {
    signal(SIGINT, mySigintHandler);
    ros::init(argc, argv, "drive_with_encoders", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("sensor_packet", 1, sensorCallback);
    ros::ServiceClient diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    signal(SIGINT, mySigintHandler);
    GridPerceptor gp(n);
    robot = Robot(diffDrive, gp);

    driveSquare(1., 1);

    return 0;
}

