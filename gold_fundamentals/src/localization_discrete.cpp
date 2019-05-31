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

void localization_demo() {
    //while not localized:
    //  observe cell
    //  estimate configuration
    //  choose action based on cell

    DiscreteLocalizer* localizer = new DiscreteLocalizer;

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