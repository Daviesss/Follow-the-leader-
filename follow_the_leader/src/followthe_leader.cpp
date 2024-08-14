#include "followthe_leader.hpp"
#include <chrono>
#include <memory>
#include <utility>
#include <iostream>

using namespace std::chrono_literals;

Followleader::Followleader() : Node("follow_the_leader_robot") {
   using std::placeholders::_1;
   laser_scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 1, std::bind(&Followleader::laser_callback, this, _1));
   vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
   timer_ = this->create_wall_timer(50ms, std::bind(&Followleader::control_cycle, this));
   laser_scan = std::numeric_limits<float>::infinity(); // Initialize laser_scan
   threshold_ = 0.9; // meters
   linear_vel = 0.1;
   angular_vel = 0.2;
}

// Laser subscription method
void Followleader::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  // Get message from the laser scanner sensor
  laser_scan = msg->ranges[msg->ranges.size() / 2];
  laser_left = msg->ranges[0]; //left angle scan
  laser_right = msg->ranges[msg->ranges.size() - 1]; // Right angle scan
  //calculating 90 degree 
  RCLCPP_INFO(this->get_logger(), "Laser scan message: %f",laser_right,laser_left);
}

void Followleader::close_to_robot(){
   velocity_message.linear.x = -0.2;
   vel_pub->publish(velocity_message);
   RCLCPP_INFO(this->get_logger(), "The robot is retreating with velocity: %f", -linear_vel);
}


void Followleader::from_the_robot(){
   velocity_message.linear.x = linear_vel;
   vel_pub->publish(velocity_message);
   RCLCPP_INFO(this->get_logger(), "The robot is moving forward with velocity: %f", linear_vel);
}



// Control method
void Followleader::control_cycle() {
  // Calculate the distance from the object/robot and tell the robot to follow 
   if (laser_scan < 0.42) {
     velocity_message.linear.x = 0.0;
     vel_pub->publish(velocity_message);
     RCLCPP_INFO(this->get_logger(), "The robot has stopped: %f");
  } 


   else if (laser_scan > 0.42){
     from_the_robot();
  }

  //TODO: if the object/obstacle if at the right the robot should follow the obstacle .Same as that of the left
  //To implement it in here
   else if (laser_left < threshold_){
    velocity_message.linear.x = 0.0; // Stop forward movement
    velocity_message.angular.z = angular_vel; // Turn right
    vel_pub->publish(velocity_message);
    RCLCPP_INFO(this->get_logger(), "Turning right to avoid obstacle on the left.");
  }

   else if (laser_right < threshold_){
    velocity_message.linear.x = 0.0; // Stop forward movement
    velocity_message.angular.z = -angular_vel; // Turn left
    vel_pub->publish(velocity_message);
    RCLCPP_INFO(this->get_logger(), "Turning left to avoid obstacle on the left.");
  }
  
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Followleader>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
