#include "Robot.h"

int direction;

const double Robot::LOOPRATE = 30;
const double Robot::ENCODER_STEPS_PER_REVOLUTION = M_PI * 2.0;
const double Robot::LASER_OFFSET = 0.12;
const double Robot::MAX_SPEED = 10.; //15.625
const double Robot::MIN_SPEED = 1.;
const double Robot::RADIUS = 0.17425;
const double Robot::SAFETY_DISTANCE = RADIUS + 0.1;
const double Robot::TRACK = 0.258;
const double Robot::WHEEL_RADIUS = 0.032;


Robot::Robot() :
    controller(Robot::MAX_SPEED, -Robot::MAX_SPEED, 0.4, 0.0, 0.0)
    {}

Robot::Robot(ros::ServiceClient diff_drive, GridPerceptor gp) : Robot() {
    diff_drive = diff_drive;
    gp = gp;
}

void Robot::diffDrive(double left, double right) {
    ROS_INFO("diffDrive requested %lf %lf", left, right);
    if (left != 0 && fabs(left) < MIN_SPEED) {
        ROS_INFO("speed to low. adjusting to +- %lf", MIN_SPEED);
        left = MIN_SPEED * sgn(left);
    }
    if (right != 0 && fabs(right) < MIN_SPEED) {
        ROS_INFO("speed to low. adjusting to +- %lf", MIN_SPEED);
        right = MIN_SPEED * sgn(right);
    }
    create_fundamentals::DiffDrive srv;
    srv.request.left = left;
    srv.request.right = right;
    diff_drive.call(srv);
}

void Robot::turnRandom() {
    int degree = rand() % 360 - 180;
    double radiant = degree * M_PI / 180.0;
    turn(radiant);
}

void Robot::brake() {
    ROS_INFO("braking: diffDrive 0 0");
    create_fundamentals::DiffDrive srv;
    srv.request.left = 0.0f;
    srv.request.right = 0.0f;
    diff_drive.call(srv);
}

void Robot::drive(double distance) {

    while (sensorData == NULL) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }

    double setpoint = sensorData->encoderLeft +
                      distance * Robot::ENCODER_STEPS_PER_REVOLUTION / (M_PI * 2.0 * Robot::WHEEL_RADIUS);
    double position = sensorData->encoderLeft;
    double prev_encoder = sensorData->encoderLeft;
    double out = 1.0;

    ros::Rate loop_rate(LOOPRATE);
    while (ros::ok() && fabs(setpoint - position) > 0.1) {
        ros::spinOnce();

        double diff_abs = sensorData->encoderLeft - prev_encoder;
        double diff = (sensorData->encoderLeft - prev_encoder) * sgn(out);
        position += diff;

        out = controller.calculate(setpoint, position, 1.0 / LOOPRATE);
        diffDrive(out, out);

        ROS_INFO("enc:%lf, pre:%lf, goal:%lf, pos:%lf, diff%lf, speed:%lf", sensorData->encoderLeft, prev_encoder,
                 setpoint, position, diff, out);
        prev_encoder = sensorData->encoderLeft;
        loop_rate.sleep();
    }

    brake();
    controller.reset();
}


void Robot::turn(double angle) {

    double distance = angle * Robot::TRACK / 2.0;

    while (sensorData == NULL) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }

    double setpoint = sensorData->encoderLeft +
                      distance * Robot::ENCODER_STEPS_PER_REVOLUTION / (M_PI * 2.0 * Robot::WHEEL_RADIUS);
    double position = sensorData->encoderLeft;
    double prev_encoder = sensorData->encoderLeft;
    double out = 1.0;

    ros::Rate loop_rate(LOOPRATE);
    while (ros::ok() && fabs(setpoint - position) > 0.1) {
        ros::spinOnce();

        double diff = fabs(sensorData->encoderLeft - prev_encoder) * sgn(out);
        position += diff;

        out = controller.calculate(setpoint, position, 1.0 / LOOPRATE);
        diffDrive(-out, out);

        ROS_INFO("enc:%lf, pre:%lf, goal:%lf, pos:%lf, diff%lf, speed:%lf", sensorData->encoderLeft,
                 prev_encoder, setpoint, position, diff, out);
        prev_encoder = sensorData->encoderLeft;
        loop_rate.sleep();
    }

    brake();
    controller.reset();
}

Robot::~Robot() {}