//* Header file importing all necessary libraries and definitions for detecting cuboids from RGB and depth images
#ifndef SUN_RGBD_EXAMPLE_CLASS_HPP  // Header guard to prevent multiple inclusions
#define SUN_RGBD_EXAMPLE_CLASS_HPP

// C++ includes
#include <stdio.h>
#include <iostream> // The iostream library is an object-oriented library that provides input and output functionality using streams
#include <fstream> // Input/output stream class to operate on files.

// Include Eigen linear algebra library
// Eigen needs to be added before opencv2 eigen can be included see https://github.com/opencv/opencv/issues/17366"
#include <Eigen/Core>
#include <Eigen/Dense>

// Include OpenCV computer vision library
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp> // Image processing tools
#include <opencv2/highgui/highgui.hpp> // GUI tools
#include <opencv2/core/eigen.hpp> // OpenCV utility tools for the Eigen C++ library

// Thirdparty includes
#include "ros2_tictoc_profiler/profiler.hpp" // A simple wall-timer profiler by Daniel Maturana (when he as at CMU), modified to be used as a stand-alone, library-only ros2 package

//* ROS2 includes
//* std_msgs in ROS 2 https://docs.ros2.org/foxy/api/std_msgs/index-msg.html
#include "rclcpp/rclcpp.hpp"
using std::placeholders::_1; //* TODO why this is suggested in official tutorial

// Includes from this package
#include "ros2_monocular_object_detection/sun_rgbd.h"
#include "ros2_monocular_object_detection/detect_cuboid_bbox/detect_cuboid_bbox.h"
#include "ros2_monocular_object_detection/plane_detection.h"

#define pass (void)0 // Python's equivalent of "pass" i.e. no operation

//* Class defintions and declarations
class SUNRGBDObjectDetector : public rclcpp::Node
{
    public:
    //* Variables for manipulating paths
    // std::string packagePath = "ros2_test/src/ros2_monocular_object_detection/"; //! HARDCODED but reserved for future use
    std::string pathToDataset; // Variable that holds the path to the SUN RGB-D dataset
    std::string imageFolder;
	std::string depthFolder;
	std::string labelFolder;
	std::string calibFolder;
	std::string outputFolder;
	std::string outputImgFolder;
	std::string indexFile;
	std::string objDimFile;
    
    //* Global work variables
    std::vector<std::string> vImageId;
    int totalFrameNumber; // Total number of images
    
    // Define objects from other classes
    DatasetSunRGBD dataLoader; // Define a dataloader for the sun_rgbd dataset
    detect_cuboid_bbox objectDetector; // 3D cuboid detector class
    PlaneDetection planeDetector; // To detect parametric planes from depth data
    
    //* Class constructors and destructors
    SUNRGBDObjectDetector(); // Constructor
    ~SUNRGBDObjectDetector(); // Destructor

    //* Methods
    void detectCuboidsInOneImage(int frameIndex); // TODO write a short description


    
}; // Remember this ; is required after each class definitions


#endif