#include "Robot.h"


const double Robot::LOOPRATE = 60;
const double Robot::ENCODER_STEPS_PER_REVOLUTION = M_PI * 2.0;
const double Robot::LASER_OFFSET = 0.12;
const double Robot::MAX_SPEED = 10.; //15.625
const double Robot::MIN_SPEED = 1.;
const double Robot::RADIUS = 0.17425;
const double Robot::SAFETY_DISTANCE = RADIUS + 0.1;
const double Robot::TRACK = 0.258;
const double Robot::WHEEL_RADIUS = 0.032;


Robot::Robot() {
    ros::NodeHandle n;
    this->controller = PID(Robot::MAX_SPEED, -Robot::MAX_SPEED, 0.4, 0.0, 0.0);
    this->diff_drive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    this->sub_sensor = n.subscribe("sensor_packet", 1, &Robot::sensorCallback, this);
    this->position = T_CARTESIAN_COORD(0.0, 0.0);
    this->positionGoal = T_CARTESIAN_COORD(0.0, 0.0);
    this->theta = 0;
    this->thetaGoal = nan("");
}

void Robot::diffDrive(double left, double right) {
    //ROS_INFO("diffDrive requested %lf %lf", left, right);
    if (left != 0 && fabs(left) < MIN_SPEED) {
        //ROS_INFO("speed to low. adjusting to +- %lf", MIN_SPEED);
        left = MIN_SPEED * sgn(left);
    }
    if (right != 0 && fabs(right) < MIN_SPEED) {
        //ROS_INFO("speed to low. adjusting to +- %lf", MIN_SPEED);
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
    ros::Duration(1).sleep();
}

void Robot::drive(double distance) {

    T_CARTESIAN_COORD dir;
    dir.x = cos(this->theta) * distance;
    dir.y = sin(this->theta) * distance;

    T_CARTESIAN_COORD goal = this->position + dir;

    this->driveTo(goal);
}


void Robot::turn(double angle) {

    this->turnTo(fmod(this->theta + angle + 2 * M_PI, 2 * M_PI));
}

void Robot::calculatePosition(const create_fundamentals::SensorPacket::ConstPtr &oldData,
                              const create_fundamentals::SensorPacket::ConstPtr &newData) {
    if (!oldData || !newData)
        return;

    double deltaLeft = (newData->encoderLeft - oldData->encoderLeft) * Robot::WHEEL_RADIUS;
    double deltaRight = (newData->encoderRight - oldData->encoderRight) * Robot::WHEEL_RADIUS;

    if (fabs(deltaRight - deltaLeft) < std::numeric_limits<float>::epsilon()) {
        this->position.x += deltaLeft * cos(this->theta);
        this->position.y += deltaRight * sin(this->theta);
    } else {
        double theta = (deltaRight - deltaLeft) / Robot::TRACK;
        double d = (deltaRight + deltaLeft) / 2.0;
        double r = d / theta;

        this->position.x += r * sin(this->theta + theta) - r * sin(this->theta);
        this->position.y += -r * cos(this->theta + theta) + r * cos(this->theta);
        this->theta = fmod(this->theta + theta + (M_PI * 2.0), (M_PI * 2.0));
    }
    ROS_INFO("x:%lf, y:%lf, theta:%lf", this->position.x, this->position.y, this->theta);

}

double Robot::angleDelta(double theta) {
    double delta = theta - this->theta;

    if (delta > M_PI)
        delta -= 2.0 * M_PI;
    if (delta < -M_PI)
        delta += 2.0 * M_PI;

    return delta;
}

void Robot::turnTo(double theta) {
    this->thetaGoal = theta;

    ros::Rate loop_rate(LOOPRATE);
    while (ros::ok() && !this->reachedTheta()){
        ros::spinOnce();
        loop_rate.sleep();
    }

    this->brake();
}

void Robot::driveTo(T_CARTESIAN_COORD goal) {
    this->positionGoal = goal;

    ros::Rate loop_rate(LOOPRATE);
    while (ros::ok() && !this->reachedGoal()){
        ros::spinOnce();
        loop_rate.sleep();
    }

    this->brake();
}

bool Robot::reachedGoal() {
    return (this->positionGoal - this->position).magnitude() < 0.05;
}

bool Robot::reachedTheta() {
    if (fabs(this->thetaGoal-this->theta) < 0.02)
        this->thetaGoal = nan("");

    return isnan(this->thetaGoal);
}

void Robot::spin() {
    if(this->reachedTheta())
        return;

    PID control = PID(Robot::MAX_SPEED, -Robot::MAX_SPEED, 12, 0.0, 0.0);
    double error = angleDelta(this->thetaGoal);
    double out = control.calculate(error, 0.0, 1 / LOOPRATE);
    diffDrive(-out, out);

}

void Robot::steer() {

    if(this->reachedGoal())
        return;

    PID control = PID(Robot::MAX_SPEED, Robot::MIN_SPEED, 12, 0.0, 0.0);
    PID control2 = PID(Robot::MAX_SPEED, 0.0, 15, 0.0, 0.0);

    T_CARTESIAN_COORD error = this->positionGoal - this->position;

    double out = control.calculate(error.magnitude(), 0.0, 1.0 / LOOPRATE);
    double turn = control2.calculate(angleDelta(error.theta()), 0.0, 1.0 / LOOPRATE);

    if (out > 2 * Robot::MAX_SPEED) {
        if (turn > 0) {
            diffDrive(out - turn, out);
        } else {
            diffDrive(out, out + turn);
        }
    } else {
        if (turn > 0) {
            diffDrive(out, out + turn);
        } else {
            diffDrive(out - turn, out);

        }
    }
}

void Robot::sensorCallback(const create_fundamentals::SensorPacket::ConstPtr &msg) {
    //ROS_INFO("left encoder: %lf, right encoder: %lf", msg->encoderLeft, msg->encoderRight);
    calculatePosition(this->sensorData, msg);

    this->sensorData = msg;

    steer();
    spin();
}

Robot::~Robot() {}