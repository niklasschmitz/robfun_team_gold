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


void turn(double angle) {

    double distance = angle * Robot::TRACK / 2.0;
    const int RATE = 30;

    while (robot.sensorData == NULL) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }

    PID pid = PID(1.0 / RATE, Robot::MAX_SPEED, -Robot::MAX_SPEED, 0.3, 0.0, 0.0);

    double setpoint = robot.sensorData->encoderRight +
                      distance * Robot::ENCODER_STEPS_PER_REVOLUTION / (M_PI * 2.0 * Robot::WHEEL_RADIUS);
    double position = robot.sensorData->encoderRight;
    double prev_encoder = robot.sensorData->encoderRight;
    double out = 1.0;

    ros::Rate loop_rate(RATE);
    while (ros::ok() && fabs(setpoint - position) > 0.1) {
        ros::spinOnce();

        double diff_abs = robot.sensorData->encoderRight - prev_encoder;
        double diff = (robot.sensorData->encoderRight - prev_encoder) * sgn(out);
        position += diff;

        out = pid.calculate(setpoint, position);
        robot.diffDrive(-out, out);

        ROS_INFO("enc:%lf, pre:%lf, goal:%lf, pos:%lf, diff%lf, speed:%lf", robot.sensorData->encoderRight,
                 prev_encoder, setpoint, position, diff, out);
        prev_encoder = robot.sensorData->encoderRight;
        loop_rate.sleep();
    }

    robot.diffDrive(0.0, 0.0);
}


int main(int argc, char **argv) {
    signal(SIGINT, mySigintHandler);
    ros::init(argc, argv, "drive_with_encoders", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("sensor_packet", 1, sensorCallback);
    ros::ServiceClient diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    signal(SIGINT, mySigintHandler);
    robot = Robot(diffDrive);

    // turn a quarter circle or 90 degrees
    turn(M_PI_2);

    return 0;
}
