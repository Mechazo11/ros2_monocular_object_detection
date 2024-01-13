/*
* Adopted from: https://github.com/benchun123/monocular_object_detection
* Author: Azmyin Md. Kamal
* Version: 1.0
* Date: 01/12/2024
* Compatible for ROS2 Humble

*/

//* ROS2 includes
//* std_msgs in ROS 2 https://docs.ros2.org/foxy/api/std_msgs/index-msg.html
#include "ros2_monocular_object_detection/cuboid_class.hpp"

//* Class defintion

//* main function
int main(int argc, char **argv){
    rclcpp::init(argc, argv); // Always the first line, initialize this node
    
    //* Declare a node object
    // auto node = std::make_shared<MonocularMode>(); 
    
    // TODO something equivalent to a rospy.SpingOnce() method to manually run through the image sequences.

    // rclcpp::Rate rate(20); // Set the desired update rate (e.g., 10 Hz)

    // rclcpp::spin(node); // Blocking node
    rclcpp::shutdown();
    return 0;
}

