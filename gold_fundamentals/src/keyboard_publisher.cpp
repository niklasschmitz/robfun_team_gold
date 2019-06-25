#include "ros/ros.h"
#include <cmath>
#include <csignal>
#include <cstdlib>
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include "utils/Robot.h"
#include "utils/tools.h"
#include "utils/GridPerceptor.h"
#include "gold_fundamentals/ExecutePlan.h"
#include "utils/geometry.h"
#include <iostream>
#include "std_msgs/String.h"
#include <unistd.h>
#include <termios.h>

Robot* robot;

void mySigintHandler(int sig) {
    ROS_INFO("exiting.. sig:%d", sig);
    robot->brake();

    ros::shutdown();
    delete(robot);
}

char getch() {
    char buf = 0;
    struct termios old = {0};
    if (tcgetattr(0, &old) < 0)
        perror("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0)
        perror("tcsetattr ICANON");
    if (read(0, &buf, 1) < 0)
        perror ("read()");
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0)
        perror ("tcsetattr ~ICANON");
    return (buf);
}

int main(int argc, char **argv) {
    signal(SIGINT, mySigintHandler);
    ros::init(argc, argv, "keyboard_pub", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;
    ROS_INFO("press key");
    signal(SIGINT, mySigintHandler);
    ros::Publisher keyboard_pub = n.advertise<std_msgs::String>("keyboard", 1);;
    std_msgs::String message;
    while(ros::ok) {
        message.data = getch();
        keyboard_pub.publish(message);
    }
    return 0;
}

