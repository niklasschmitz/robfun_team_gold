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

bool wallInFront() {
    return true; //TODO
}

maze::Cell observe_cell() {
    maze::Cell observation;
    //TODO

    return observation;
}

void localization_demo() {
    //while not localized:
    //  observe cell
    //  estimate configuration
    //  choose action based on cell

    DiscreteLocalizer *localizer = new DiscreteLocalizer();
    // TODO: make sure map is received already

    // initial action is (0,0,0)
    RobotConfiguration action;

    // while not localized
    while (localizer->candidates.size() > 1) {
        // observe cell (4 possible walls)
        maze::Cell observation = observe_cell();

        // estimate configuration
        localizer->estimateConfiguration(action, observation);

        // choose action based on cell
        // randomness makes this more robust against
        // getting stuck in a maze with a loop
        while (wallInFront()) {
            int direction = rand() % 4;
            robot->turn(direction * M_PI_2);
        }
        // drive to next cell
        robot->drive(localizer->maze->CELL_SIDE_LENGTH);


        // if no candidate is left, restart estimation
        if (localizer->candidates.size() == 0) {
            localizer->populateCandidates();
        }
    }

}

int main(int argc, char **argv) {
    signal(SIGINT, mySigintHandler);
    ros::init(argc, argv, "localization_discrete", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;
    signal(SIGINT, mySigintHandler);
    robot = new Robot();
    robot->align();

    localization_demo();

    delete (robot);

    return 0;
}