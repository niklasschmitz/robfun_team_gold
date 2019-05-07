#include "Robot.h"


const double Robot::LOOPRATE = 30;
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
    this->position.x = 0.0;
    this->position.y = 0.0;
    this->theta = 0.0;
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

    ros::Rate loop_rate(LOOPRATE);
    while (ros::ok() && fabs(setpoint - sensorData->encoderLeft) > 0.1) {
        ros::spinOnce();

        double out = controller.calculate(setpoint, sensorData->encoderLeft, 1.0 / LOOPRATE);
        diffDrive(out, out);
        ROS_INFO("enc:%lf, goal:%lf, speed:%lf", sensorData->encoderLeft, setpoint, out);

        loop_rate.sleep();
    }

    brake();
    controller.reset();
}


void Robot::turn(double angle) {

    double distance = -angle * Robot::TRACK / 2.0;

    while (sensorData == NULL) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }

    double setpoint = sensorData->encoderLeft +
                      distance * Robot::ENCODER_STEPS_PER_REVOLUTION / (M_PI * 2.0 * Robot::WHEEL_RADIUS);

    ros::Rate loop_rate(LOOPRATE);
    while (ros::ok() && fabs(setpoint - sensorData->encoderLeft) > 0.1) {
        ros::spinOnce();


        double out = controller.calculate(setpoint, sensorData->encoderLeft, 1.0 / LOOPRATE);
        diffDrive(out, -out);
        ROS_INFO("enc:%lf, goal:%lf, speed:%lf", sensorData->encoderLeft, setpoint, out);

        loop_rate.sleep();
    }

    brake();
    controller.reset();
}

void Robot::calculatePosition(const create_fundamentals::SensorPacket::ConstPtr &oldData,
                              const create_fundamentals::SensorPacket::ConstPtr &newData) {
    if (!oldData || !newData)
        return;

    double deltaLeft = oldData->encoderLeft - newData->encoderLeft;
    double deltaRight = oldData->encoderRight - newData->encoderRight;

    if (fabs(deltaRight - deltaLeft) < std::numeric_limits<float>::epsilon() * 10.0) {
        this->position.x += deltaLeft * Robot::WHEEL_RADIUS * cos(this->theta);
        this->position.y += deltaRight * Robot::WHEEL_RADIUS * sin(this->theta);
    } else {
        double theta = (deltaRight - deltaLeft) * Robot::WHEEL_RADIUS / Robot::TRACK;
        double d = (deltaRight + deltaLeft) * Robot::WHEEL_RADIUS / 2.0;
        double r = d / theta;

        this->position.x += r * sin(this->theta + theta) - r * sin(this->theta);
        this->position.y += r * cos(this->theta + theta) + r * cos(this->theta);
        this->theta = fmod(this->theta + theta + 360.0, 360);
    }
    ROS_INFO("x:%lf, y:%lf, theta:%lf", this->position.x, this->position.y, this->theta);

}

void Robot::sensorCallback(const create_fundamentals::SensorPacket::ConstPtr &msg) {
    ROS_INFO("left encoder: %lf, right encoder: %lf", msg->encoderLeft, msg->encoderRight);
    calculatePosition(this->sensorData, msg);

    this->sensorData = msg;
}

Robot::~Robot() {}