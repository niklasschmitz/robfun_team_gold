#include "Robot.h"

#include "geometry.h"
#include "Probability.h"


const double Robot::LOOPRATE = 100;
const double Robot::ENCODER_STEPS_PER_REVOLUTION = M_PI * 2.0;
const double Robot::LASER_OFFSET = 0.11;
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
    this->sub_laser = n.subscribe("scan_filtered", 1, &Robot::laserCallback, this);
    this->thetaGoal = nan("");
    this->sensorTime = ros::Time::now();
    this->resetPosition();
    this->pose_pub = n.advertise<gold_fundamentals::Pose>("pose", 1);
    this->bigChangeInPose = true;
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

void Robot::resetPosition(){
    this->position = T_VECTOR2D(0.0,0.0);
    this->theta = M_PI_2;
}

void Robot::publishPosition(){
    gold_fundamentals::Pose msg;
    msg.orientation = (int)round(this->theta / M_PI_2) + 3 % 4;
    msg.row = round(this->position.x);
    msg.column = round(this->position.y);
    this->pose_pub.publish(msg);
}

void Robot::calculatePosition(const create_fundamentals::SensorPacket::ConstPtr &oldData,
                              const create_fundamentals::SensorPacket::ConstPtr &newData) {
    if (!oldData || !newData) { return; }

    double deltaLeft = (newData->encoderLeft - oldData->encoderLeft) * Robot::WHEEL_RADIUS;
    double deltaRight = (newData->encoderRight - oldData->encoderRight) * Robot::WHEEL_RADIUS;

    double deltaX = 0;
    double deltaY = 0;
    double deltaTheta = 0;

    double oldX = this->position.x;
    double oldY = this->position.y;
    double oldTheta = this->theta;

    if (fabs(deltaRight - deltaLeft) < std::numeric_limits<float>::epsilon() * 10.0) {
        double d = (deltaRight + deltaLeft) / 2.0;
        deltaX = d * cos(this->theta);
        deltaY = d * sin(this->theta);
        this->position.x += deltaX;
        this->position.y += deltaY;
    } else {
        double d = (deltaRight + deltaLeft) / 2.0;
        double theta = (deltaRight - deltaLeft) / Robot::TRACK;
        double r = d / theta;

        deltaX = r * sin(this->theta + theta) - r * sin(this->theta);
        deltaY = -r * cos(this->theta + theta) + r * cos(this->theta);


        // TODO: check this on correctness!
        //deltaTheta = Probability::diffAngle(this->theta, theta);
        double newTheta = fmod(this->theta + theta + (M_PI * 2.0), (M_PI * 2.0));
        deltaTheta = newTheta - oldTheta;

        this->position.x += deltaX;
        this->position.y += deltaY;
        this->theta = newTheta;//fmod(this->theta + theta + (M_PI * 2.0), (M_PI * 2.0));
    }

    double newX = this->position.x;
    double newY = this->position.y;
    double newTheta = this->theta;

    this->bigChangeInPose = fabs(deltaX) > 0.025 || fabs(deltaY) > 0.025 || fabs(deltaTheta) > 0.1;

    // see if we should update the filter
    if (this->bigChangeInPose && this->particleFilter.initialized) {
//        ROS_INFO("we have a big change");
        pfMutex.lock();
        this->particleFilter.sampleMotionModel(oldX, oldY, oldTheta, newX, newY, newTheta);
        pfMutex.unlock();
    } else {
//        ROS_INFO("no change");
    }

    Particle* best_hyp = this->particleFilter.getBestHypothesis();
    if(best_hyp != NULL) {
        ROS_INFO("current pos: x=%lf, y=%lf, th=%lf, w=%lf", best_hyp->x, best_hyp->y, best_hyp->theta, best_hyp->weight);
    }

    this->publishPosition();
    //ROS_INFO("x:%lf, y:%lf, theta:%lf", this->position.x, this->position.y, this->theta);
}

double Robot::angleDelta(double theta) {
    double delta = theta - this->theta;

    if (delta > M_PI) { delta -= 2.0 * M_PI; }
    if (delta < -M_PI) { delta += 2.0 * M_PI; }

    return delta;
}

void Robot::turnTo(double theta) {
    this->thetaGoal = fmod(theta + (M_PI * 2.0), (M_PI * 2.0));
//    ROS_INFO("turning");
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

bool Robot::goalBehindRobot(T_VECTOR2D goal, T_VECTOR2D error){
    return goal * error < 0.0;
}

void Robot::steer() {
    if (this->path.size() == 0)
        return;

    while (path.size() > 1 && this->isCloseTo(path.front())) {
        path.pop();
    }

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

        ROS_INFO("angleDelta:%lf, turn:%lf", angleDelta(error.theta()), turn);

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
    while (lines.size() == 0 || lines.size() == 1) {
        turn(M_PI_2);
        ros::spinOnce();
        lines = gp.getLines();
        //TODO fallback
        ROS_INFO("ALIGNMENT ERROR: not enough walls detected");
    }

    T_VECTOR2D diff_vec = gp.getAlignmentTargetPositionDifference();
    T_VECTOR2D goal_vec = this->position + diff_vec;//.rotate(this->theta);

    ROS_INFO("our position is , x = %lf y = %lf", this->position.x, this->position.y);
    ROS_INFO("the goal is , x = %lf y = %lf", goal_vec.x, goal_vec.y);

    resetPosition();

    // drive to middle of cell
    if(!goal_vec.isvalid()) {
        turn(M_PI_2);
    } else {
        turnTo(diff_vec.theta());
        driveTo(goal_vec);
    }

    if(reachedGoal(goal_vec)) {
        // align to wall
        alignToWall();
    }

    this->resetPosition();
}


void Robot::alignToWall() {
    std::vector<T_RATED_LINE> lines;
    lines = gp.getLines();

    // get line with most inliers
    T_RATED_LINE best_line = gp.getLineWithMostInliers();

    std::vector<T_VECTOR2D> angles;

    ros::spinOnce();

    while(!best_line.isvalid()) {
        turn(M_PI_2);
        best_line = gp.getLineWithMostInliers();
    }

    //angles.push_back(angle);

    double angle = T_VECTOR2D::angleBetweenRobotAndVector(best_line.line.u);
    double turn_value = angle - M_PI_2;
    //ROS_INFO("angle %lf", angle * 180/M_PI);


    // align to the wall
    while(fabs(turn_value) > 5/180.0*M_PI) {
        turn(angle - M_PI_2);
        best_line = gp.getLineWithMostInliers();
        angle = T_VECTOR2D::angleBetweenRobotAndVector(best_line.line.u);
        turn_value = angle - M_PI_2;
    }

    ROS_INFO("terminated");
}

void Robot::sensorCallback(const create_fundamentals::SensorPacket::ConstPtr &msg) {
    ros::Time time = ros::Time::now();
    this->timeDelta = (time - this->sensorTime).toSec();
    this->sensorTime = time;

    // in calc_pos is also the particle filter
    calculatePosition(this->sensorData, msg);
    this->sensorData = msg;

//    ROS_INFO("left:%u, right:%u", this->sensorData->bumpLeft, this->sensorData->bumpRight);
    if(this->sensorData->bumpLeft || this->sensorData->bumpRight){
        ROS_INFO("OH NO!");
        this->brake();
        exit(1);
    }

    spin();
    steer();
}

void Robot::laserCallback(const sensor_msgs::LaserScan::ConstPtr &laserScan) {
    // NOTE: there is also a laserCallback in the gridPerceptor
//    ROS_INFO("robot laser callback");

    // if the robot has moved, update the filter
    if (this->bigChangeInPose && this->particleFilter.initialized) {
        ROS_INFO("updating particle filter via laser");
        pfMutex.lock();

        // correction step
        this->particleFilter.measurementModel(laserScan);
        // resample the particles
        this->particleFilter.resample();
        pfMutex.unlock();
        this->bigChangeInPose = false;
    }
}

Robot::~Robot() {}