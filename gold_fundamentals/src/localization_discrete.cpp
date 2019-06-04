#include "ros/ros.h"
#include <csignal>
#include <cmath>
#include "create_fundamentals/DiffDrive.h"
#include "utils/Robot.h"
#include "utils/GridPerceptor.h"
#include "utils/DiscreteLocalizer.h"

Robot *robot;


void mySigintHandler(int sig) {
    ROS_INFO("exiting.. sig:%d", sig);
    robot->brake();

    delete (robot);

    ros::shutdown();
}

// TODO: this is just a demo. might also be part of GridPerceptor
bool wallInFront() {
    std::vector<T_RATED_LINE> lines = robot->gp.getLines();

    // for every line
    for (int line_idx = 0; line_idx < lines.size(); ++line_idx) {
        T_LINE line = lines[line_idx].line;

        // check if is loosely close to orthogonal to robot (which is facing along x axis)
        // and not too far away (<0.6m)
        // u is normalized
        T_VECTOR2D origin;
        if (fabs(line.u.x) < 0.3 && distBetweenLineAndPoint(line, origin) < 0.6) {
            return true;
        }
    }

    return false;
}

/**
 * checks for presence of each wall
 * right is the direction the robot faces
 * when this method is called. the labeling
 * continues clock-wise
 * @return Cell observation
 */
maze::Cell observe_cell() {
    maze::Cell observation;

    for (int i = 0; i < 4; ++i) {
        // check presence of wall in front
        observation.set(i, wallInFront());

        // turn 90 degrees
        robot->turn(M_PI_2);
    }

    return observation;
}

gold_fundamentals::Pose localization_demo() {
    //while not localized:
    //  observe cell
    //  estimate configuration
    //  choose action based on cell

    DiscreteLocalizer *localizer = new DiscreteLocalizer();

    // wait until map is received
    while (!localizer->received_map) {
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }

    localizer->populateCandidates();

    int local_direction = 0;

    // while not localized
    while (localizer->candidates.size() > 1) {
        // observe cell (4 possible walls)
        maze::Cell observation = observe_cell();

        // estimate configuration
        localizer->estimateConfiguration(local_direction, observation);

        // turn towards free direction to explore.
        // randomness makes this more robust against
        // getting stuck in a maze with a loop
        while (wallInFront()) {
            int direction = rand() % 4;
            robot->turn(direction * M_PI_2);
            local_direction += direction;
            local_direction %= 4;
        }
        // drive to next cell
        robot->drive(localizer->maze->CELL_SIDE_LENGTH);

        // if no candidate is left, restart estimation
        if (localizer->candidates.size() == 0) {
            localizer->populateCandidates();
        }
    }
    // localized. play mario song
    robot->playSong(1);

    gold_fundamentals::Pose pose = localizer->candidates[0];
    return pose;
}


int main(int argc, char **argv) {
    signal(SIGINT, mySigintHandler);
    ros::init(argc, argv, "localization_discrete", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;
    signal(SIGINT, mySigintHandler);
    robot = new Robot();
    robot->align();

    // localize
    gold_fundamentals::Pose pose = localization_demo();

    // publish the position
    while (ros::ok()) {
        robot->pose_pub.publish(pose);
        ros::Duration(0.5).sleep();
    }

    delete (robot);

    return 0;
}