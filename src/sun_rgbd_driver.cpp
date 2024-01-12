/*
* Adopted from: 
* Author: Azmyin Md. Kamal
* Version: 1.0
* Date: 01/12/2024
* Compatible for ROS2 Humble

* NOTE
    * We define all definitions and logic to drive the node here ??

*/

//* ROS2 includes
//* std_msgs in ROS 2 https://docs.ros2.org/foxy/api/std_msgs/index-msg.html
#include "rclcpp/rclcpp.hpp"



//* main
int main(int argc, char **argv){
    rclcpp::init(argc, argv); // Always the first line, initialize this node
    
    //* Declare a node object
    // auto node = std::make_shared<MonocularMode>(); 
    
    // rclcpp::Rate rate(20); // Set the desired update rate (e.g., 10 Hz)

    // rclcpp::spin(node); // Blocking node
    rclcpp::shutdown();
    return 0;
}

