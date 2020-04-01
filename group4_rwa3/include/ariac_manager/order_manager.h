#pragma once

#include <list>
#include <map>
#include <string>
#include <iostream>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/Order.h>
#include <osrf_gear/Proximity.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include "sensor.h"
#include "robot_controller.h"

class AriacOrderManager {
public:
    AriacOrderManager();
    ~AriacOrderManager();
    void OrderCallback(const osrf_gear::Order::ConstPtr& order_msg);
    void break_beam_callback(const osrf_gear::Proximity::ConstPtr& msg);
    void LogicalCamera4Callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg);	
    void ExecuteOrder();
    std::string GetProductFrame(std::string product_type);
    std::map<std::string, std::list<std::pair<std::string,geometry_msgs::Pose>>> GetOrder();
    bool PickAndPlace(std::pair<std::string,geometry_msgs::Pose> object_prop,int agvnum);
    void SubmitAGV(int num);

private:
    ros::NodeHandle order_manager_nh_;
    ros::Subscriber order_subscriber_;
    std::vector<osrf_gear::Order> received_orders_;
    AriacSensorManager camera_;
    RobotController arm1_;
//    RobotController arm2_;
    tf::TransformListener part_tf_listener_;
    std::pair<std::string,geometry_msgs::Pose> product_type_pose_;
    std::string object;
    std::map<std::string, std::vector<std::string>> product_frame_list_;
    osrf_gear::Order order_;
    osrf_gear::Proximity beam_;
    int flag = 0;
    int flag_order=0;
    ros::Subscriber break_beam_subscriber_;
    ros::Subscriber camera_4_subscriber_;
    std::vector<std::string> cameraParts;
};

