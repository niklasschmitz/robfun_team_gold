#include "ros/ros.h"
#include <cmath>
#include <csignal>
#include <cstdlib>
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"

const double ROBOT_WHEEL_RADIUS = 0.032;
const double ROBOT_TRACK = 0.235;
const double ROBOT_ANGULAR_SPEED = 5.0; //max: 15.625
const double ROBOT_MAX_ANGULAR_SPEED = 15.625;
const double ENCODER_STEPS_PER_REVOLUTION = M_PI * 2.0;

ros::ServiceClient diffDrive;
bool obstacle = false; //for testing purposes only

class Variables {
public:
    double pos_desired;
    double pos_actual;
    double prev_encoderLeft;
    double prev_encoderRight;

    //PID
    double kP;
    double kD;

    double last_error;

    Variables() {
        pos_desired = 0;
        pos_actual = 0;
        prev_encoderLeft = 0; // would be better to init to current encoder values when calling main first time
        prev_encoderRight = 0;

        //PID
        kP = 5;
        kD = 1;

        last_error = 0;
    }

    ~Variables() {

    }
};

Variables vars;


void brake() {
    ROS_INFO("braking: diffDrive 0 0");
    create_fundamentals::DiffDrive srv;
    srv.request.left = 0;
    srv.request.right = 0;
    diffDrive.call(srv);
}


double angularToLinear(double angular_vel) {
    return angular_vel * ROBOT_WHEEL_RADIUS;
}

double linearToAngular(double linear_vel) {
    return linear_vel / ROBOT_WHEEL_RADIUS;
}

double distanceToTime(double distance, double speed) {
    return std::abs(distance / angularToLinear(speed));
}


int sgn(double val) {
    if (val > 0) { return 1; }
    if (val < 0) { return -1; }
    return 0;
}


// angle in radians
void turn(double angle) {
    brake();

    double distance = angle * ROBOT_TRACK / 2;
    double time = distanceToTime(distance, ROBOT_ANGULAR_SPEED);
    ROS_INFO("turning: diffDrive -10 10 %lf %lf", angle, time);
    create_fundamentals::DiffDrive srv;
    srv.request.left = -ROBOT_ANGULAR_SPEED * sgn(angle);
    srv.request.right = ROBOT_ANGULAR_SPEED * sgn(angle);
    diffDrive.call(srv);

    ros::Duration(time).sleep();
    brake();
}

void drive(double distance) {
    //TODO
}

/*
 * Calls service to set the angular velocities of the wheels given linear velocities
 *
*/
void setVelocity(double wheelL_linear_vel, double wheelR_linear_vel) {
    ROS_INFO("driving: w1 lin_vel %lf, w2 lin_vel %lf", wheelL_linear_vel, wheelR_linear_vel);
    create_fundamentals::DiffDrive srv;

    // calc angular velocities, limit to max speed
    double wheelL_angular_vel = std::max(std::min(linearToAngular(wheelL_linear_vel), ROBOT_ANGULAR_SPEED), -ROBOT_ANGULAR_SPEED);
    double wheelR_angular_vel = std::max(std::min(linearToAngular(wheelR_linear_vel), ROBOT_ANGULAR_SPEED), -ROBOT_ANGULAR_SPEED);

    // call service
    srv.request.left = wheelL_angular_vel;
    srv.request.right = wheelR_angular_vel;
    diffDrive.call(srv);
}

void sensorCallback(const create_fundamentals::SensorPacket::ConstPtr &msg) {
    ROS_INFO("left encoder: %lf, right encoder: %lf", msg->encoderLeft, msg->encoderRight);

    // calc distance driven (or maybe calc velocity?)
    double enc_left = msg->encoderLeft;
    double enc_right = msg->encoderRight;

    double wheelLeft_radiansDriven = 2 * M_PI / ENCODER_STEPS_PER_REVOLUTION * (enc_left - vars.prev_encoderLeft);
    double wheelLeft_distanceDriven = wheelLeft_radiansDriven * ROBOT_WHEEL_RADIUS;

    double wheelRight_radiansDriven = 2 * M_PI / ENCODER_STEPS_PER_REVOLUTION * (enc_right - vars.prev_encoderRight);
    double wheelRight_distanceDriven = wheelRight_radiansDriven * ROBOT_WHEEL_RADIUS;

    vars.pos_actual += (wheelLeft_distanceDriven + wheelRight_distanceDriven) / 2;

    vars.prev_encoderLeft = enc_left;
    vars.prev_encoderRight = enc_right;

    double error = vars.pos_desired - vars.pos_actual;
    double delta_error = error - vars.last_error;
    double vel_des = vars.kP * error + vars.kD * delta_error;
    setVelocity(vel_des, vel_des);
}


void mySigintHandler(int sig) {
    ROS_INFO("exiting..");
    brake();

    ros::shutdown();
}


// side_length: side length of square in metres
// iterations: number of repetitions
void driveSquare(double side_length, int iterations) {
    // 4 turns make 1 square
    for (int i = 0; i < iterations * 4; ++i) {
        drive(side_length);
        ros::Duration(0.5).sleep();
        turn(M_PI_2);
        ros::Duration(0.5).sleep();
    }
}


int main(int argc, char **argv) {
    signal(SIGINT, mySigintHandler);
    ros::init(argc, argv, "square_with_encoders", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("sensor_packet", 1, sensorCallback);
    diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    signal(SIGINT, mySigintHandler);

    ros::spin();
    mySigintHandler(0);

    return 0;
}
