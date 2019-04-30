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
    //ROS_INFO("left encoder: %lf, right encoder: %lf", msg->encoderLeft, msg->encoderRight);
    robot.sensorData = msg;
}

void mySigintHandler(int sig) {
    ROS_INFO("exiting.. sig:%d", sig);
    robot.brake();

    ros::shutdown();
}

void align() {

}


int main(int argc, char **argv) {
    signal(SIGINT, mySigintHandler);
    ros::init(argc, argv, "align", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;
    ros::Subscriber sub_sensor = n.subscribe("sensor_packet", 1, sensorCallback);
    ros::ServiceClient diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    signal(SIGINT, mySigintHandler);
    GridPerceptor gp(n);
    robot = Robot(diffDrive, gp);

    //align();
    ros::spin();

    return 0;
}

