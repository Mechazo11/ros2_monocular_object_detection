/*
* Adopted from: https://github.com/benchun123/monocular_object_detection
* Author: Azmyin Md. Kamal
* Version: 1.0
* Date: 01/12/2024
* Compatible for ROS2 Humble

*/

//* ROS2 includes
//* std_msgs in ROS 2 https://docs.ros2.org/foxy/api/std_msgs/index-msg.html
#include "ros2_monocular_object_detection/sun_rgbd_example_class.hpp"
// #include "tictoc_profiler/profiler.hpp" // use the tictoc_profiler time statistics package

//* Class defintion

//* main function
int main(int argc, char **argv){
    rclcpp::init(argc, argv); // Always the first line, initialize this node
    
    //* Declare a node object
    auto node = std::make_shared<SUNRGBDObjectDetector>();
    rclcpp::WallRate loop_rate(20); // Loop at 10 hz
    int frameIdx = 0;

    ca::Profiler::enable();

    while (rclcpp::ok() && frameIdx < node->totalFrameNumber) {
        
        node->detectCuboidsInOneImage(frameIdx);
        frameIdx++;
        rclcpp::spin_some(node); //? is this equivalent to rclpy.spinOnce()?
        loop_rate.sleep();
        
        // TODO delete if works
        // Main loop to go through all the images in the sequence
        // for (int frameIdx = 0; frameIdx < node->totalFrameNumber; frameIdx++){
        //     node->detectCuboidsInOneImage(frameIdx);
        //     // std::cout<<"After detect cuboid images\n";
        //     rclcpp::spin_some(node); //? is this equivalent to rclpy.spinOnce()?
        // }
    }
    
    node.reset();; // Calls destructor of SUNRGBDObjectDetector class
    ca::Profiler::print_aggregated(std::cout); // Print out all time statistics
    rclcpp::shutdown(); // Shutdown this ROS 2 
    return 0;
}


/*
To be deleted
    
// rclcpp::Rate rate(20); // Set the desired update rate (e.g., 10 Hz)
// rclcpp::spin(node); // Blocking node, only for testing
*/
