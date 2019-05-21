#include "Robot.h"

const double Robot::LASER_OFFSET = 0.11;
const double Robot::LOOPRATE = 100;
const double Robot::MAX_SPEED = 10.; //15.625
const double Robot::MIN_SPEED = 1.;
const double Robot::RADIUS = 0.17425;
const double Robot::SAFETY_DISTANCE = RADIUS + 0.1;
const double Robot::TRACK = 0.258;
const double Robot::WHEEL_RADIUS = 0.032;


Robot::Robot() {
    ros::NodeHandle n;
    this->turnControl = PID(Robot::MAX_SPEED, -Robot::MAX_SPEED, 12, 0.0, 0.0);
    this->speedControl = PID(Robot::MAX_SPEED - 3, Robot::MIN_SPEED, 12, 0.0, 0.0);
    this->steerControl = PID(Robot::MAX_SPEED, -Robot::MAX_SPEED, 15, 0.0, 0.0);
    this->steerMaxControl = PID(Robot::MAX_SPEED, -Robot::MAX_SPEED, 12, 0.0, 0.0);
    this->diff_drive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    this->store_song = n.serviceClient<create_fundamentals::StoreSong>("store_song");
    this->play_song = n.serviceClient<create_fundamentals::PlaySong>("play_song");
    this->sub_sensor = n.subscribe("sensor_packet", 1, &Robot::sensorCallback, this);
    this->pose_pub = n.advertise<gold_fundamentals::Pose>("pose", 1);
    this->sensorTime = ros::Time::now();
    this->resetPosition();
    this->storeSong();
}

void Robot::storeSong() {
    create_fundamentals::StoreSong srv;
    srv.request.number = 0;
    srv.request.song.push_back(36);
    srv.request.song.push_back(64);
    store_song.call(srv);
}

void Robot::playSong(int number) {
    create_fundamentals::PlaySong srv;
    srv.request.number = number;
    play_song.call(srv);
}

void Robot::diffDrive(double left, double right) {
    if (left != 0 && fabs(left) < MIN_SPEED) {
        left = MIN_SPEED * sgn(left);
    }
    if (right != 0 && fabs(right) < MIN_SPEED) {
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
    create_fundamentals::DiffDrive srv;
    srv.request.left = 0.0f;
    srv.request.right = 0.0f;
    diff_drive.call(srv);
    ros::Duration(0.5).sleep();
}

void Robot::drive(double distance) {
    T_VECTOR2D goal = this->position + T_VECTOR2D(distance, 0.0).rotate(this->theta);
    this->driveTo(goal);
}


void Robot::turn(double angle) {
    this->turnTo(this->theta + angle);
}

void Robot::resetPosition() {
    this->position = T_VECTOR2D(0.0, 0.0);
    this->theta = M_PI_2;
}

void Robot::publishPosition() {
    gold_fundamentals::Pose msg;
    msg.orientation = (int) round(this->theta / M_PI_2) % 4;
    msg.row = round(this->position.x);
    msg.column = round(this->position.y);
    this->pose_pub.publish(msg);
}

void Robot::calculatePosition(const create_fundamentals::SensorPacket::ConstPtr &oldData,
                              const create_fundamentals::SensorPacket::ConstPtr &newData) {
    if (!oldData || !newData) {
        return;
    }

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
        this->theta = normalizeAngle(this->theta + theta);
    }

    this->publishPosition();
}

double Robot::angleDelta(double theta) {
    double delta = normalizeAngle(theta) - this->theta;

    if (delta > M_PI) {
        delta -= 2.0 * M_PI;
    }
    if (delta < -M_PI) {
        delta += 2.0 * M_PI;
    }

    return delta;
}

void Robot::turnTo(double theta) {
    double thetaGoal = normalizeAngle(theta);

    ros::Rate loop_rate(LOOPRATE);
    create_fundamentals::SensorPacket_<std::allocator<void> >::ConstPtr last = this->sensorData;
    while (ros::ok() && !this->reachedTheta(thetaGoal)) {
        if (last != this->sensorData) {
            last = this->sensorData;
            spin(thetaGoal);
        }
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
    ros::Rate loop_rate(LOOPRATE);
    create_fundamentals::SensorPacket_<std::allocator<void> >::ConstPtr last = this->sensorData;
    while (ros::ok() && !path.empty()) {
        if (last != this->sensorData) {
            last = this->sensorData;

            while (path.size() > 1 && this->isCloseTo(path.front())) {
                path.pop();
            }

            if (path.size() == 1) {
                if (this->reachedGoal(path.front())) {
                    path.pop();
                    return;
                }
                this->drivePID(path.front());
            } else {
                this->driveMAX(path.front());
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    this->brake();
}

void Robot::drivePID(T_VECTOR2D goal){
    T_VECTOR2D error = goal - this->position;

    double out = speedControl.calculate(error.magnitude(), 0.0, this->timeDelta);
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
}

void Robot::driveMAX(T_VECTOR2D checkpoint){
    T_VECTOR2D error = checkpoint - this->position;

    double speed = Robot::MAX_SPEED;
    double turn = steerMaxControl.calculate(angleDelta(error.theta()), 0.0, this->timeDelta);

    if (turn > 0) {
        diffDrive(speed - turn, speed);
    } else {
        diffDrive(speed, speed + turn);
    }
}

bool Robot::isCloseTo(T_VECTOR2D point) {
    return (point - this->position).magnitude() < 0.4;
}

bool Robot::reachedGoal(T_VECTOR2D goal) {
    return (goal - this->position).magnitude() < 0.02;
}

bool Robot::reachedTheta(double thetaGoal) {
    return fabs(thetaGoal - this->theta) < 0.02;
}

void Robot::spin(double thetaGoal) {
    if (this->reachedTheta(thetaGoal))
        return;

    double error = angleDelta(thetaGoal);
    double out = turnControl.calculate(error, 0.0, this->timeDelta);
    diffDrive(-out, out);

}

void Robot::align() {
    std::vector<T_RATED_LINE> lines;
    lines = gp.getLines();
    while (lines.size() == 0 || lines.size() == 1) {
        turn(M_PI_2);
        ros::spinOnce();
        lines = gp.getLines();
        //TODO fallback
        ROS_INFO("ALIGNMENT ERROR: not enough walls detected");
    }

    T_VECTOR2D diff_vec = gp.getAlignmentTargetPositionDifference().rotate(this->theta);
    T_VECTOR2D goal_vec = this->position + diff_vec;

    if (!goal_vec.isvalid()) {
        ROS_INFO("ALIGNMENT ERROR: goal vec is invalid");
    } else {
        turnTo(diff_vec.theta());
        driveTo(goal_vec);
        this->resetPosition();
        alignToWall();
        ROS_INFO("aligned");
    }

    this->resetPosition();
}


void Robot::alignToWall() {
    T_RATED_LINE best_line = gp.getLineWithMostInliers();

    while (!best_line.isvalid()) {
        turn(M_PI_2);
        best_line = gp.getLineWithMostInliers();
    }

    double angle = T_VECTOR2D::angleBetweenRobotAndVector(best_line.line.u);

    while (fabs(angle) > 1.0 / 180.0 * M_PI) {
        turn(angle);
        best_line = gp.getLineWithMostInliers();
        angle = T_VECTOR2D::angleBetweenRobotAndVector(best_line.line.u);
    }
}

void Robot::sensorCallback(const create_fundamentals::SensorPacket::ConstPtr &msg) {
    ros::Time time = ros::Time::now();
    this->timeDelta = (time - this->sensorTime).toSec();
    this->sensorTime = time;

    calculatePosition(this->sensorData, msg);
    this->sensorData = msg;

    if (this->sensorData->bumpLeft || this->sensorData->bumpRight) {
        ROS_INFO("OH NO!");
        this->playSong(0);
        this->brake();
        exit(1);
    }
}

Robot::~Robot() {}