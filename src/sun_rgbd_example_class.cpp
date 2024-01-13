// TODO need one line descriptor

// Includes
#include "ro2_monocular_object_detector/sun_rgbd_example_class.hpp"


SUNRGBDCuboidObjectDetector::SUNRGBDCuboidObjectDetector() :Node("cuboid-detector")
{
/*
Class constructor
*/

/*
Pseudocode
* Pass the entire path
* Load files as needed
*/

RCLCPP_INFO(this->get_logger(), "\nMonocular Object detector from point-plane-objects-SLAM paper started\n");

// Declare command-line parameters
this->declare_parameter("path_to_sunrgbd_dataset_arg", "not_given"); // Path to SUN RGB-D dataset sample as used in benchun`s repository

// Initialize some intial values
pathToDataset = "not_set";

//! RESUME FROM HERE

}


SUNRGBDCuboidObjectDetector::~SUNRGBDCuboidObjectDetector()
{
/*
Class destructor, release resources and gracefully exit
*/
}