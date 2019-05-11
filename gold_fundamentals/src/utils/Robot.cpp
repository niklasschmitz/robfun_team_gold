#include "Robot.h"

#include "geometry.h"

const double Robot::LOOPRATE = 100;
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
    //this->gp = GridPerceptor();
    this->diff_drive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    this->sub_sensor = n.subscribe("sensor_packet", 1, &Robot::sensorCallback, this);
    this->position = T_VECTOR2D(0.0, 0.0);
    this->theta = M_PI_2;
    this->thetaGoal = nan("");
    this->sensorTime = ros::Time::now();
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
    ros::Duration(0.5).sleep();
}

void Robot::drive(double distance) {
    T_VECTOR2D dir;
    dir.x = cos(this->theta) * distance;
    dir.y = sin(this->theta) * distance;

    T_VECTOR2D goal = this->position + dir;

    this->driveTo(goal);
}


void Robot::turn(double angle) {
    this->turnTo(this->theta + angle);
}

void Robot::calculatePosition(const create_fundamentals::SensorPacket::ConstPtr &oldData,
                              const create_fundamentals::SensorPacket::ConstPtr &newData) {
    if (!oldData || !newData) { return; }

    double deltaLeft = (newData->encoderLeft - oldData->encoderLeft) * Robot::WHEEL_RADIUS;
    double deltaRight = (newData->encoderRight - oldData->encoderRight) * Robot::WHEEL_RADIUS;

    if (fabs(deltaRight - deltaLeft) < 1.0e-6) {
        double d = (deltaRight + deltaLeft) / 2.0;
        this->position.x += d * cos(this->theta);
        this->position.y += d * sin(this->theta);
    } else {
        double d = (deltaRight + deltaLeft) / 2.0;
        double theta = (deltaRight - deltaLeft) / Robot::TRACK;
        double r = d / theta;

        this->position.x += r * sin(this->theta + theta) - r * sin(this->theta);
        this->position.y += -r * cos(this->theta + theta) + r * cos(this->theta);
        this->theta = fmod(this->theta + theta + (M_PI * 2.0), (M_PI * 2.0));
    }

    ROS_INFO("x:%lf, y:%lf, theta:%lf", this->position.x, this->position.y, this->theta);
}

double Robot::angleDelta(double theta) {
    double delta = theta - this->theta;

    if (delta > M_PI) { delta -= 2.0 * M_PI; }
    if (delta < -M_PI) { delta += 2.0 * M_PI; }

    return delta;
}

void Robot::turnTo(double theta) {
    this->thetaGoal = fmod(theta + (M_PI * 2.0), (M_PI * 2.0));

    ros::Rate loop_rate(LOOPRATE);
    while (ros::ok() && !this->reachedTheta()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    this->brake();
}

void Robot::driveTo(T_VECTOR2D goal) {
    std::queue<T_VECTOR2D> path;
    path.push(goal);

    this->followPath(path);
}

void Robot::followPath(std::queue<T_VECTOR2D> path) {
    this->path = path;

    ros::Rate loop_rate(LOOPRATE);
    while (ros::ok() && this->path.size() > 0) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    this->brake();
}

bool Robot::isCloseTo(T_VECTOR2D point) {
    return (point - this->position).magnitude() < 0.4;
}

bool Robot::reachedGoal(T_VECTOR2D goal) {
    return (goal - this->position).magnitude() < 0.02;
}

bool Robot::reachedTheta() {
    if (fabs(this->thetaGoal - this->theta) < 0.02)
        this->thetaGoal = nan("");

    return isnan(this->thetaGoal);
}

void Robot::spin() {
    if (this->reachedTheta())
        return;

    PID turnControl = PID(Robot::MAX_SPEED, -Robot::MAX_SPEED, 12, 0.0, 0.0);
    double error = angleDelta(this->thetaGoal);
    double out = turnControl.calculate(error, 0.0, this->timeDelta);
    diffDrive(-out, out);

}

void Robot::steer() {
    if (this->path.size() == 0)
        return;

    while (path.size() > 1 && this->isCloseTo(path.front())) {
        path.pop();
    }

    ROS_INFO("goal  x:%lf, y=%lf", this->path.front().x, this->path.front().y);

    if (this->path.size() == 1) {
        if (this->reachedGoal(path.front())) {
            path.pop();
            return;
        }

        PID driveControl = PID(Robot::MAX_SPEED - 3, Robot::MIN_SPEED, 12, 0.0, 0.0);
        PID steerControl = PID(Robot::MAX_SPEED, -Robot::MAX_SPEED, 15, 0.0, 0.0);

        T_VECTOR2D error = this->path.front() - this->position;

        double out = driveControl.calculate(error.magnitude(), 0.0, this->timeDelta);
        double turn = steerControl.calculate(angleDelta(error.theta()), 0.0, this->timeDelta);

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

    } else {
        PID steerControl = PID(Robot::MAX_SPEED, -Robot::MAX_SPEED, 12, 0.0, 0.0);
        T_VECTOR2D error = path.front() - this->position;

        double speed = Robot::MAX_SPEED;
        double turn = steerControl.calculate(angleDelta(error.theta()), 0.0, this->timeDelta);

        if (turn > 0) {
            diffDrive(speed - turn, speed);
        } else {
            diffDrive(speed, speed + turn);
        }
    }
}

void Robot::align() {
    std::vector<T_RATED_LINE> lines;
    lines = gp.getLines();
    T_VECTOR2D x_axis = T_VECTOR2D(1.0, 0);
    T_VECTOR2D robot_position = T_VECTOR2D(0.0, 0.0);
    if (lines.size() == 0) {
        ROS_INFO("no lines");
    } else {
        for (int i = 0; i < lines.size(); ++i) {
            //ROS_INFO("%lf", distBetweenLineAndPoint(lines[i].line, robot_position));
//            double dir_vec_x = lines[i].line.u.x;
//            double dir_vec_y = lines[i].line.u.y;

            //ROS_INFO("before mirror: u.x %lf, u.y %lf", lines[i].line.u.x, lines[i].line.u.y);
            //T_VECTOR2D mirrored = T_VECTOR2D::mirror(lines[i].line.u, x_axis);
            // map the vector to only be in the first 180°
            //if((dir_vec_x<))
            //ROS_INFO("after mirror: u.x %lf, u.y %lf", mirrored.x, mirrored.y);

        }
    }
}

void Robot::sensorCallback(const create_fundamentals::SensorPacket::ConstPtr &msg) {
    ros::Time time = ros::Time::now();
    this->timeDelta = (time - this->sensorTime).toSec();
    this->sensorTime = time;

    calculatePosition(this->sensorData, msg);
    this->sensorData = msg;

    spin();
    steer();
}

Robot::~Robot() {}