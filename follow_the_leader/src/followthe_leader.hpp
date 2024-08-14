//Declaration .hpp file
#pragma once 
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>
#include <iostream>



class Followleader : public rclcpp::Node{
public:
    Followleader();
    void control_cycle(); //compute the follow leader algorithm in here
    void close_to_robot();
    void from_the_robot();
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::Twist velocity_message;
    //implementation variables
    double distance;
    double threshold_; 
    double linear_vel;
    double angular_vel; 
    float laser_scan;
    double laser_right;
    double laser_left;
};
