#include "ros/ros.h"
#include <cmath>
#include <csignal>
#include <cstdlib>
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include "utils/Robot.h"
#include "utils/tools.h"
#include "utils/GridPerceptor.h"

#include "utils/geometry.h"

Robot* robot;


void mySigintHandler(int sig) {
    ROS_INFO("exiting.. sig:%d", sig);
    robot->brake();

    ros::shutdown();
    delete(robot);
}

void align_robot() {
    //robot->align();
}


int main(int argc, char **argv) {
    signal(SIGINT, mySigintHandler);
    ros::init(argc, argv, "align", ros::init_options::NoSigintHandler);
    robot = new Robot();
    signal(SIGINT, mySigintHandler);


//    T_VECTOR2D v2 = T_VECTOR2D(-1/sqrt(2),1/sqrt(2)); // the wall
//    ROS_INFO("angle1 %lf", T_VECTOR2D::angleBetweenRobotAndVector(v2));
////    ROS_INFO("angle1 %lf", robot.angleDelta(v2));
//    //ROS_INFO("angle1 %lf", T_VECTOR2D::signedAngleBetweenVectors(v1, v2));
//
//    v2 = T_VECTOR2D(-1/sqrt(2),-1/sqrt(2));
////    ROS_INFO("angle1 %lf", robot.angleDelta(v2.theta()));
//    ROS_INFO("angle2 %lf", T_VECTOR2D::angleBetweenRobotAndVector(v2));
//    //ROS_INFO("angle2 %lf", T_VECTOR2D::signedAngleBetweenVectors(v1, v2));
//
//    v2 = T_VECTOR2D(1/sqrt(2),-1/sqrt(2));
//    ROS_INFO("angle3 %lf", T_VECTOR2D::angleBetweenRobotAndVector(v2));
//    //ROS_INFO("angle3 %lf", T_VECTOR2D::signedAngleBetweenVectors(v1, v2));
//
//    v2 = T_VECTOR2D(1/sqrt(2),1/sqrt(2));
//    ROS_INFO("angle4 %lf", T_VECTOR2D::angleBetweenRobotAndVector(v2));
//    //ROS_INFO("angle4 %lf", T_VECTOR2D::signedAngleBetweenVectors(v1, v2));
//
//    v2 = T_VECTOR2D(1,0); // the wall
//    ROS_INFO("angle00 %lf", T_VECTOR2D::angleBetweenRobotAndVector(v2));
//
//    v2 = T_VECTOR2D(-1,0); // the wall
//    ROS_INFO("angle10 %lf", T_VECTOR2D::angleBetweenRobotAndVector(v2));
//
//    T_VECTOR2D vec1 = T_VECTOR2D(0,1);
//    T_VECTOR2D vec2 = T_VECTOR2D(0,-1);
//    ROS_INFO("vector angle %lf", T_VECTOR2D::angleBetweenVectors(vec1, vec2));
//
//    T_VECTOR2D sup_vec1 = T_VECTOR2D(0,0);
//    T_VECTOR2D sup_vec2 = T_VECTOR2D(0,1);
//    T_VECTOR2D dir_vec1 = T_VECTOR2D(1/sqrt(2),1/sqrt(2));
//    T_VECTOR2D dir_vec2 = T_VECTOR2D(1/sqrt(2),-1/sqrt(2));
//
//    T_LINE line1 = T_LINE(sup_vec1, dir_vec1);
//    T_LINE line2 = T_LINE(sup_vec2, dir_vec2);
//
//    T_VECTOR2D intersection_vec = intersectionOfTwoLines(line1, line2);
//
//    ROS_INFO("intersection x %lf, y %lf", intersection_vec.x, intersection_vec.y);

    while(ros::ok) {
        align_robot();
        ros::spinOnce();
    }

    delete(robot);
    return 0;
}

