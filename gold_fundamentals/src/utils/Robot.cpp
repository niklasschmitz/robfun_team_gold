#include "Robot.h"

#include "geometry.h"
#include "Probability.h"

#define RELOCALISATION_DETECTION 1

const double Robot::LOOPRATE = 100;
const double Robot::LASER_OFFSET = 0.11;
const double Robot::MAX_SPEED = 7.; //15.625
const double Robot::MIN_SPEED = 1.;
const double Robot::RADIUS = 0.17425;
const double Robot::SAFETY_DISTANCE = RADIUS + 0.1;
const double Robot::TRACK = 0.258;
const double Robot::WHEEL_RADIUS = 0.032;


Robot::Robot() {
    ros::NodeHandle n;
    this->turnControl = PID(Robot::MAX_SPEED, -Robot::MAX_SPEED, 10, 0.0, 0.0);
    this->speedControl = PID(Robot::MAX_SPEED, Robot::MIN_SPEED, 12, 0.0, 0.0);
    this->steerControl = PID(Robot::MAX_SPEED, -Robot::MAX_SPEED, 7, 0.0, 0.0);
    this->steerMaxControl = PID(Robot::MAX_SPEED, -Robot::MAX_SPEED, 7, 0.0, 0.0);
    this->diff_drive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    this->store_song = n.serviceClient<create_fundamentals::StoreSong>("store_song");
    this->play_song = n.serviceClient<create_fundamentals::PlaySong>("play_song");
    this->sub_sensor = n.subscribe("sensor_packet", 1, &Robot::sensorCallback, this);
    this->sub_laser = n.subscribe("scan", 1, &Robot::laserCallback, this);
    this->sub_keyboard = n.subscribe("keyboard", 1, &Robot::keyboardCallback, this);
    this->pose_pub = n.advertise<gold_fundamentals::Pose>("pose", 1);
    this->sensorTime = ros::Time::now();
    this->obstacle = true;
    this->storeSong();
    this->playSong(0);
    this->localized = false;
    this->update_theta = false;
    this->doNotAbort = true;
    this->resetPosition();
    this->gold_count = 0;
    this->keyboard_received = false;
}

void Robot::storeSong() {
    create_fundamentals::StoreSong song0; //Warning
    song0.request.number = 0;
    song0.request.song = {36, 64};
    store_song.call(song0);

    create_fundamentals::StoreSong song1; //Mario 1UP
    song1.request.number = 1;
    song1.request.song = {88, 8, 91, 8, 100, 8, 96, 8, 98, 8, 103, 16};
    store_song.call(song1);

    create_fundamentals::StoreSong song2; //Indiana Jones
    song1.request.number = 2;
    song1.request.song = {64, 16, 65, 8, 67, 8, 72, 48, 62, 16, 64, 8, 65, 40, 65, 16, 67, 8, 69, 8, 77, 48, 69, 16, 71, 8, 72, 24, 74, 24, 76, 24};
    store_song.call(song1);
}

void Robot::playSong(int number) {
    create_fundamentals::PlaySong srv;
    srv.request.number = number;
    play_song.call(srv);
}

void Robot::diffDrive(double left, double right) {
    if (left != 0) {
        left = absMin(left, MIN_SPEED);
    }
    if (right != 0) {
        right = absMin(right, MIN_SPEED);
    }
    create_fundamentals::DiffDrive srv;
    srv.request.left = left;
    srv.request.right = right;
    diff_drive.call(srv);
}

void Robot::wander() {
    while (ros::ok()) {
        ros::spinOnce();
        if (this->obstacle) {
            this->turnRandom();
        } else {
            this->diffDrive(Robot::MAX_SPEED, Robot::MAX_SPEED);
        }
    }
}

bool Robot::isLocalized() {
    double variance = this->particleFilter.calculateParticleVariance();
    double threshold = 10;
    return variance < threshold && this->particleFilter.initialized;
}

void Robot::localize() {
    ROS_INFO("starting localization");
    this->localized = false;
    this->update_theta = false;
    while (ros::ok() && !this->isLocalized()) {
        ros::spinOnce();

        if (this->obstacle_left) {
            this->turnRandomRight();
        } else if (this->obstacle_right) {
            this->turnRandomLeft();
        } else if (this->obstacle_front) {
            this->turnRandom();
        } else {
            this->diffDrive(Robot::MAX_SPEED, Robot::MAX_SPEED);
        }
    }
    this->update_theta = true;
    brake();
    this->driveCenterCell();
    this->playSong(1);

    // we have a localisation proposal, decrease wanderer obstacle sensitivity for plan execution?
//    SAFETY_DISTANCE = SAFETY_DISTANCE - 0.05;

    Particle *bestPart = this->particleFilter.getBestHypothesis();
    this->theta = bestPart->theta;
    ROS_INFO("my position is: cell_x: %lf, cell_y: %lf, theta_deg: %lf", bestPart->x / 0.8, bestPart->y / 0.8,
             bestPart->theta * 180.0 / M_PI);

//    this->localized = true;
}

void Robot::turnRandom() {
    int degree = rand() % 360 - 180;
    double radiant = degree * M_PI / 180.0;
    turn(radiant);
}

//turns random left 0 to 90°
void Robot::turnRandomLeft() {
    int degree = rand() % 90;
    double radiant = degree * M_PI / 180.0;
    turn(radiant);
}

//turns random right 0 to 90°
void Robot::turnRandomRight() {
    int degree = rand() % 90 - 90;
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
    this->theta = 0.;
}

void Robot::publishPosition() {
    T_VECTOR2D position = this->getCellxy();
    gold_fundamentals::Pose msg;
    msg.orientation = (int) round(this->theta / M_PI_2) % 4;
    msg.row = position.y;
    msg.column = position.x;
    this->pose_pub.publish(msg);
    //ROS_INFO("x:%lf, y:%lf, theta:%lf", this->position.x, this->position.y , this->theta);
}

void Robot::calculatePosition(const create_fundamentals::SensorPacket::ConstPtr &oldData,
                              const create_fundamentals::SensorPacket::ConstPtr &newData) {
    if (!oldData || !newData) {
        return;
    }

    double deltaLeft = (newData->encoderLeft - oldData->encoderLeft) * Robot::WHEEL_RADIUS;
    double deltaRight = (newData->encoderRight - oldData->encoderRight) * Robot::WHEEL_RADIUS;

    double oldX = this->position.x;
    double oldY = this->position.y;
    double oldTheta = this->theta;

    if (fabs(deltaRight - deltaLeft) < std::numeric_limits<float>::epsilon() * 10.0) {
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

    double newX = this->position.x;
    double newY = this->position.y;
    double newTheta = this->theta;

    if (this->particleFilter.initialized) {
        this->particleFilter.sampleMotionModel(oldX, oldY, oldTheta, newX, newY, newTheta);
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

    this->turnControl.reset();
    this->brake();
}

void Robot::driveTo(T_VECTOR2D goal, bool ignore_localized) {
    std::queue<T_VECTOR2D> path;
    path.push(goal);

    this->followPath(path, ignore_localized);
}

// returns nearest cell center
T_VECTOR2D Robot::getCell() {
    int x = round((this->position.x - MAZE_SIDE_LENGTH_2) / MAZE_SIDE_LENGTH);
    int y = round((this->position.y - MAZE_SIDE_LENGTH_2) / MAZE_SIDE_LENGTH);
    return T_VECTOR2D(x * MAZE_SIDE_LENGTH + MAZE_SIDE_LENGTH_2, y * MAZE_SIDE_LENGTH + MAZE_SIDE_LENGTH_2);
}

// returns robot cell position (for pose pub)
T_VECTOR2D Robot::getCellxy() {
    int x = round((this->position.x - MAZE_SIDE_LENGTH_2) / MAZE_SIDE_LENGTH);
    int y = round((this->position.y - MAZE_SIDE_LENGTH_2) / MAZE_SIDE_LENGTH);
    return T_VECTOR2D((this->particleFilter.oc_grid.max_cells_y - 1) - y, x);
}

void Robot::executePlan(std::vector<int> plan) {

    T_VECTOR2D dist(MAZE_SIDE_LENGTH_2, 0);
    T_VECTOR2D next = this->getCell();

    std::queue<T_VECTOR2D> path;

    for (int i = 0; i < plan.size(); i++) {
        next = next + dist.rotate(plan[i] * M_PI_2);
        next = next + dist.rotate(plan[i] * M_PI_2);
        path.push(next);
    }

    double to = plan.front() * M_PI_2;
    this->turnTo(to);
    this->followPath(path);
}

void Robot::followPath(std::queue<T_VECTOR2D> path, bool ignore_localized) {
    ros::Rate loop_rate(LOOPRATE);
    create_fundamentals::SensorPacket_<std::allocator<void> >::ConstPtr last = this->sensorData;
//    while (ros::ok() && !path.empty() && (localized || ignore_localized) && doNotAbort) {
    while (ros::ok() && !path.empty() && !(this->localized && this->obstacle == true)) {
        if (last != this->sensorData) {
            last = this->sensorData;

            while (path.size() > 1 && this->isCloseTo(path.front())) {
                path.pop();
            }

            if (path.size() == 1) {
                if (this->reachedGoal(path.front())) {
                    path.pop();
                } else {
                    this->drivePID(path.front());
                }
            } else {
                this->driveMAX(path.front());
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    this->steerControl.reset();
    this->speedControl.reset();
    this->steerMaxControl.reset();
    ROS_INFO("braking");
    this->brake();

    //    ROS_INFO("loc %d, obst %d", this->localized, this->obstacle);
    // if we were localized (and executing a plan right now), we should not see any obstacles
    // if we do, we lost localisation and should reinitialise the particle filter
#if RELOCALISATION_DETECTION == 1
    if (this->localized && this->obstacle == true) {
        ROS_INFO("resetting localisation");
        this->localized = false;
//        this->obstacle = false;
        this->particleFilter.initParticlesUniform();
        this->particleFilter.resetUniformResamplingPercentage();
        this->playSong(0);
        this->localize();
        return;
    }
#endif
}

void Robot::drivePID(T_VECTOR2D goal) {
    T_VECTOR2D error = goal - this->position;

    if (fabs(angleDelta(error.theta())) > M_PI_2) {
        turn(angleDelta(error.theta()));
        return;
    }

    double out = speedControl.calculate(error.magnitude(), 0.0, this->timeDelta);
    double turn = steerControl.calculate(angleDelta(error.theta()), 0.0, this->timeDelta);

    if (2.0 * out > Robot::MAX_SPEED) {
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

void Robot::driveMAX(T_VECTOR2D checkpoint) {
    T_VECTOR2D error = checkpoint - this->position;

    if (fabs(angleDelta(error.theta())) > M_PI_2) {
        turn(angleDelta(error.theta()));
        return;
    }

    double speed = Robot::MAX_SPEED;
    double turn = steerMaxControl.calculate(angleDelta(error.theta()), 0.0, this->timeDelta);

    if (turn > 0) {
        diffDrive(speed - turn, speed);
    } else {
        diffDrive(speed, speed + turn);
    }
}

bool Robot::isCloseTo(T_VECTOR2D point) {
    return (point - this->position).magnitude() < 0.2;
}

bool Robot::reachedGoal(T_VECTOR2D goal) {
    return (goal - this->position).magnitude() < 0.05;
}

bool Robot::reachedTheta(double thetaGoal) {
    return fabs(thetaGoal - this->theta) < 0.1;
}

void Robot::spin(double thetaGoal) {
    if (this->reachedTheta(thetaGoal))
        return;

    double error = angleDelta(thetaGoal);
    double out = turnControl.calculate(error, 0.0, this->timeDelta);
    diffDrive(-out, out);

}

void Robot::digForGold() {
    this->keyboard_received = false;
    this->playSong(2);
    ros::Duration(5).sleep();
    ros::spinOnce();
    if(!this->keyboard_received){
        this->gold_count += 10;
    }

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

void Robot::keyboardCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("input received %s", msg->data.c_str());
    this->keyboard_received = true;
}

void Robot::sensorCallback(const create_fundamentals::SensorPacket::ConstPtr &msg) {
    ros::Time time = ros::Time::now();
    this->timeDelta = (time - this->sensorTime).toSec();
    this->sensorTime = time;

    // in calc_pos is also the particle filter
    calculatePosition(this->sensorData, msg);
    this->sensorData = msg;

//    ROS_INFO("left:%u, right:%u", this->sensorData->bumpLeft, this->sensorData->bumpRight);
    if (this->sensorData->bumpLeft || this->sensorData->bumpRight) {
        ROS_INFO("OH NO!");
        this->diffDrive(-1, -1);
        ros::Duration(2).sleep();
        this->playSong(0);
        this->brake();
        exit(1);
    }
}

void Robot::driveCenterCell() {
    T_VECTOR2D goal = this->getCell();
    T_VECTOR2D error = goal - this->position;
    double to = angleDelta(error.theta());

    this->turn(to);
    this->localized = true;
    this->driveTo(goal, true);
    this->turnTo(0);
}

inline double distanceEllipse(double angle) {
    const double a = Robot::SAFETY_DISTANCE - Robot::LASER_OFFSET + 0.08;
    const double b = Robot::SAFETY_DISTANCE - 0.06;

    return a * b / sqrt(pow(b * cos(angle), 2.0) + pow(a * sin(angle), 2.0));
}

void Robot::laserCallback(const sensor_msgs::LaserScan::ConstPtr &laserScan) {
    double angle = laserScan->angle_min;  // = -1.57 = -90°
    this->obstacle = false;
    this->obstacle_left = false;
    this->obstacle_front = false;
    this->obstacle_right = false;

    // from which degree from the center do we say that the obstacle is left/right
    double obstacle_side_limit = 30.0;
    // until which degree from the center do we say that the obstacle is in the front
    double obstacle_front_limit = 30.0;

    // detect if there are obstacles
    for (int i = 0; i < laserScan->ranges.size(); i++) {
        if (laserScan->ranges[i] < distanceEllipse(angle)) {

            if (angle < -obstacle_side_limit * M_PI / 180.0) {
                this->obstacle_right = true;
            }
            if (angle > obstacle_side_limit * M_PI / 180.0) {
                this->obstacle_left = true;
            }
            if (angle > -obstacle_front_limit && angle < obstacle_front_limit) {
                this->obstacle_front = true;
            }
            this->obstacle = true;
        }
        angle += laserScan->angle_increment;
    }

    if (this->particleFilter.initialized) {
        this->particleFilter.measurementModel(laserScan);
        this->particleFilter.resample();

        Particle *best_hyp = this->particleFilter.getBestHypothesis();

        if (best_hyp != NULL) {
            this->position.x = best_hyp->x;
            this->position.y = best_hyp->y;
            if (this->update_theta)
                this->theta = normalizeAngle(best_hyp->theta);
        }
    }
}

Robot::~Robot() {}