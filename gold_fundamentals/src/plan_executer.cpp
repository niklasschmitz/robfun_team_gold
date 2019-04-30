#include "ros/ros.h"
#include "gold_fundamentals/ExecutePlan.h"

bool execute(gold_fundamentals::ExecutePlan::Request &req, gold_fundamentals::ExecutePlan::Response &res) {
    for(int i = 0; i < req.plan.size(); i++)
        ROS_INFO("%d", req.plan[i]);

    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "plan_executer");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("execute_plan", execute);
    ROS_INFO("Ready to execute.");
    ros::spin();

    return 0;
}