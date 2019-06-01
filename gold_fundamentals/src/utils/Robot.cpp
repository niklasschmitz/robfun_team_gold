#include "Robot.h"

#include "geometry.h"
#include "Probability.h"
#include "time.h"


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
    this->speedControl = PID(Robot::MAX_SPEED - 3, Robot::MIN_SPEED, 12, 0.0, 0.0);
    this->steerControl = PID(Robot::MAX_SPEED, -Robot::MAX_SPEED, 10, 0.0, 0.0);
    this->steerMaxControl = PID(Robot::MAX_SPEED, -Robot::MAX_SPEED, 7, 0.0, 0.0);
    this->diff_drive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    this->store_song = n.serviceClient<create_fundamentals::StoreSong>("store_song");
    this->play_song = n.serviceClient<create_fundamentals::PlaySong>("play_song");
    this->sub_sensor = n.subscribe("sensor_packet", 1, &Robot::sensorCallback, this);
    this->sub_laser = n.subscribe("scan_filtered", 1, &Robot::laserCallback, this);
    this->pose_pub = n.advertise<gold_fundamentals::Pose>("pose", 1);
    this->sensorTime = ros::Time::now();
    this->resetPosition();
    this->obstacle = false;
    this->storeSong();
    this->playSong(0);
    this->bigChangeInPose = false;
    this->updateTheta = true;
}

void Robot::storeSong() {
    create_fundamentals::StoreSong song0;
    song0.request.number = 0;
    song0.request.song.push_back(36);
    song0.request.song.push_back(64);
    store_song.call(song0);

    create_fundamentals::StoreSong song1; //Mario 1UP
    song1.request.number = 1;
    song1.request.song.push_back(88);
    song1.request.song.push_back(8);
    song1.request.song.push_back(91);
    song1.request.song.push_back(8);
    song1.request.song.push_back(100);
    song1.request.song.push_back(8);
    song1.request.song.push_back(96);
    song1.request.song.push_back(8);
    song1.request.song.push_back(98);
    song1.request.song.push_back(8);
    song1.request.song.push_back(103);
    song1.request.song.push_back(16);
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

void Robot::wander(){
    while (ros::ok()) {
        ros::spinOnce();
        if(this->obstacle){
            this->turnRandom();
        } else {
            this->diffDrive(Robot::MAX_SPEED, Robot::MAX_SPEED);
        }
    }
}

bool Robot::isLocalized() {
    double variance = this->particleFilter.calculateParticleVariance();
    double threshold = 10;
    if (variance < threshold && this->particleFilter.initialized) {
        return true;
    }
    return false;
}

void Robot::localize() {
    this->updateTheta = false;
    while (ros::ok() && !this->isLocalized()) { //TODO: run until localized
        ros::spinOnce();
//        ROS_INFO("localized %d", this->isLocalized());
        //this->particleFilter.publishBestParticleRawLikelihood();
        //this->particleFilter.publishParticleVariance();
        if(this->obstacle_left){
            ROS_INFO("obst left");
            this->turnRandomRight();
        } else if(this->obstacle_right) {
            ROS_INFO("obst right");
            this->turnRandomLeft();
        } else if(this->obstacle_front) {
            ROS_INFO("obst front");
            this->turnRandom();
        }else {
            this->diffDrive(Robot::MAX_SPEED, Robot::MAX_SPEED);
        }
    }
    brake();
    Particle* bestPart = this->particleFilter.getBestHypothesis();
    this->updateTheta = true;
    this->theta = bestPart->theta;
    ROS_INFO("my position is: cell_x: %lf, cell_y: %lf, theta_deg: %lf", bestPart->x/0.8, bestPart->y/0.8, bestPart->theta*180.0/M_PI);
}

void Robot::turnRandom() {
    int degree = rand() % 360 - 180;
    double radiant = degree * M_PI / 180.0;
    turn(radiant);
}

//turns random left 0 to 90°
void Robot::turnRandomLeft() {
    ROS_INFO("random left");
    int degree = rand() % 90;
    double radiant = degree * M_PI / 180.0;
    turn(radiant);
}

//turns random right 0 to 90°
void Robot::turnRandomRight() {
    ROS_INFO("random right");
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
    this->theta = M_PI_2;
}

void Robot::publishPosition() {
    gold_fundamentals::Pose msg;
    msg.orientation = (int) round(this->theta / M_PI_2) % 4;
    msg.row = round((this->position.y - 0.4) / 0.8); //TODO: subtract #MapRows
    msg.column = round((this->position.x - 0.4) / 0.8);
    this->pose_pub.publish(msg);
//    ROS_INFO("x:%lf, y:%lf, theta:%lf", this->position.x, this->position.y , this->theta);
}

void Robot::calculatePosition(const create_fundamentals::SensorPacket::ConstPtr &oldData,
                              const create_fundamentals::SensorPacket::ConstPtr &newData) {
    if (!oldData || !newData) {
        return;
    }

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
        double newTheta = normalizeAngle(this->theta + theta);
        deltaTheta = newTheta - oldTheta;

        this->position.x += deltaX;
        this->position.y += deltaY;
        this->theta = newTheta;//fmod(this->theta + theta + (M_PI * 2.0), (M_PI * 2.0));
    }

    double newX = this->position.x;
    double newY = this->position.y;
    double newTheta = this->theta;

    // see if we should update the filter
    if (this->particleFilter.initialized) {
//        ROS_INFO("we have a big change");
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

    this->steerControl.reset();
    this->speedControl.reset();
    this->steerMaxControl.reset();
    this->brake();
}

void Robot::drivePID(T_VECTOR2D goal) {
    T_VECTOR2D error = goal - this->position;

    if(fabs(angleDelta(error.theta())) > 1.3){
        turn(angleDelta(error.theta()));
        return;
    }

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

void Robot::driveMAX(T_VECTOR2D checkpoint) {
    T_VECTOR2D error = checkpoint - this->position;

    if(fabs(angleDelta(error.theta())) > 1.3) {
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
    return (point - this->position).magnitude() < 0.4;
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

    // in calc_pos is also the particle filter
    calculatePosition(this->sensorData, msg);
    this->sensorData = msg;

//    ROS_INFO("left:%u, right:%u", this->sensorData->bumpLeft, this->sensorData->bumpRight);
    if(this->sensorData->bumpLeft || this->sensorData->bumpRight){
        ROS_INFO("OH NO!");
        this->diffDrive(-1, -1);
        ros::Duration(2).sleep();
        this->playSong(0);
        this->brake();
        exit(1);
    }
}

void Robot::driveCenterCell() {
    int x = round((this->position.x - 0.4) / 0.8);
    int y = round((this->position.y - 0.4) / 0.8);
    T_VECTOR2D goal(x * 0.8 + 0.4, y * 0.8 + 0.4);
    T_VECTOR2D error = goal - this->position;
    double to = angleDelta(error.theta());

    this->turn(to);
    this->driveTo(goal);
    this->turnTo(0);
}

inline double distanceEllipse(double angle) {
    const double a = Robot::SAFETY_DISTANCE - Robot::LASER_OFFSET + 0.1;
    const double b = Robot::SAFETY_DISTANCE - 0.05;

    return a * b / sqrt(pow(b * cos(angle), 2.0) + pow(a * sin(angle), 2.0));
}

void Robot::laserCallback(const sensor_msgs::LaserScan::ConstPtr &laserScan) {

    //clock_t start, end;// = clock();
    //start = clock();
    double angle = laserScan->angle_min; // = -1.57 = -90°
    this->obstacle = false;
    this->obstacle_left = false;
    this->obstacle_front = false;
    this->obstacle_right = false;

//    this->obstacle = false;

    // from which degree from the center do we say that the obstacle is left/right
    double obstacle_side_limit = 30.0;
    // until which degree from the center do we say that the obstacle is in the front
    double obstacle_front_limit = 30.0;

    for (int i = 0; i < laserScan->ranges.size(); i++) {
        if (laserScan->ranges[i] < distanceEllipse(angle)) {

            if(angle < -obstacle_side_limit * M_PI/180.0) {
                this->obstacle_right = true;
            }
            if(angle > obstacle_side_limit * M_PI/180.0) {
                this->obstacle_left = true;
            }
            if(angle > -obstacle_front_limit && angle < obstacle_front_limit) {
                this->obstacle_front = true;
            }
            this->obstacle = true;
            //break;
        }
        angle += laserScan->angle_increment;
    }


    // NOTE: there is also a laserCallback in the gridPerceptor

    // if the robot has moved, update the filter
    if (this->particleFilter.initialized) {

        struct timespec ts1, ts2;
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ts1);
        // correction step
        this->particleFilter.measurementModel(laserScan);
        // resample the particles
        this->particleFilter.resample();

        Particle* best_hyp = this->particleFilter.getBestHypothesis();

        if(best_hyp != NULL) {
            this->position.x = best_hyp->x;
            this->position.y = best_hyp->y;
            if (updateTheta)
                this->theta = normalizeAngle(best_hyp->theta);
        }

        //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ts2);

        //double dur = 1000.0 * ts2.tv_sec + 1e-6 *ts2.tv_nsec - (1000.0 * ts1.tv_sec + 1e-6 * ts1.tv_nsec);

        //ROS_INFO("delay = %lf", dur);

    }
//    end = clock();
//    double duration_sec = (end-start) / CLOCKS_PER_SEC;
//    ROS_INFO("time for obstacle detect: %lf", duration_sec);
}

Robot::~Robot() {}