/**
 *
 * Copyright [2019] <KCL King's College London>
 *
 * Author: Martin Koling (martinh.koling@kcl.ac.uk)
 * Author: Oscar Lima (oscar.lima@dfki.de)
 *
 * Unit tests for ROSPlan plan dispatch
 */

#include <ros/ros.h>
#include <thread>
#include <chrono>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <gtest/gtest.h>

#include "rosplan_dispatch_msgs/ParsingService.h"
#include "rosplan_dispatch_msgs/CompletePlan.h"
#include "rosplan_dispatch_msgs/EsterelPlan.h"
#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_dispatch_msgs/ActionFeedback.h"
#include "rosplan_dispatch_msgs/DispatchService.h"

bool runPlanningControl(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
        ros::NodeHandle nh("~");
std::string srv_name0 = "/rosplan_plan_dispatcher/cancel_dispatch";
    ros::ServiceClient client0 = nh.serviceClient<std_srvs::Empty>(srv_name0);
    std_srvs::Empty srv0;

    std::string srv_name1 = "/rosplan_problem_interface/problem_generation_server";
    ros::ServiceClient client1 = nh.serviceClient<std_srvs::Empty>(srv_name1);
    std_srvs::Empty srv1;

    std::string srv_name2 = "/rosplan_planner_interface/planning_server";
    ros::ServiceClient client2 = nh.serviceClient<std_srvs::Empty>(srv_name2);
    std_srvs::Empty srv2;

    std::string srv_name3 = "/rosplan_parsing_interface/parse_plan";
    ros::ServiceClient client3 = nh.serviceClient<std_srvs::Empty>(srv_name3);
    std_srvs::Empty srv3;

    std::string srv_name4 = "/rosplan_plan_dispatcher/dispatch_plan";
    ros::ServiceClient client4 = nh.serviceClient<rosplan_dispatch_msgs::DispatchService>(srv_name4);
    rosplan_dispatch_msgs::DispatchService srv4;

    ros::service::waitForService(srv_name1, ros::Duration(1.0));
    client1.call(srv1);

    ros::service::waitForService(srv_name2, ros::Duration(1.0));
    client2.call(srv2);

    ros::service::waitForService(srv_name3, ros::Duration(1.0));
    client3.call(srv3);

    ros::service::waitForService(srv_name4, ros::Duration(1.0));
    client4.call(srv4);

while(!srv4.response.success || !srv4.response.goal_achieved)
{
    ROS_INFO("***********************runPlanningControl: response.success=%s  response.goal_achieved=(%s)",\
                    srv4.response.success ? "true" : "false", srv4.response.goal_achieved ? "true" : "false");
    client0.call(srv0);
    client1.call(srv1);
    client2.call(srv2);
    client3.call(srv3);
    client4.call(srv4);

}


ROS_INFO("***********************runPlanningControl: response.success=%s  response.goal_achieved=(%s)",\
                    srv4.response.success ? "true" : "false", srv4.response.goal_achieved ? "true" : "false");
return true;
    }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("run_planning_control", runPlanningControl);
  ROS_INFO("Ready to add two ints.");
  ros::spin();

  return 0;
}


