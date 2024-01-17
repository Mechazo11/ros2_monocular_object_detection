// TODO need one line descriptor

// Includes
#include "ros2_monocular_object_detection/sun_rgbd_example_class.hpp"
#include "tictoc_profiler/profiler.hpp"

SUNRGBDObjectDetector::SUNRGBDObjectDetector() :Node("cuboid-detector")
{
    /*
    Class constructor
    *

    /*
    Pseudocode
    * Pass the entire path
    * Load files as needed
    */

    RCLCPP_INFO(this->get_logger(), "\nMonocular Object detector from point-plane-objects-SLAM paper started\n");

    // Declare command-line parameters
    this->declare_parameter("path_to_sunrgbd_dataset_arg", "not_given"); // Path to SUN RGB-D dataset sample as used in benchun`s repository

    // Populate path set by user from command-line
    pathToDataset = "not_set";
    rclcpp::Parameter param1 = this->get_parameter("path_to_sunrgbd_dataset_arg");
    pathToDataset = param1.as_string();

    RCLCPP_INFO(this->get_logger(), "pathToDataset %s", pathToDataset.c_str());

    imageFolder = pathToDataset + "/rgb/";
	depthFolder = pathToDataset + "/depth/";
	labelFolder = pathToDataset + "/label/";
	calibFolder = pathToDataset + "/calib/";
	outputFolder = pathToDataset + "/offline_cuboid/";
	outputImgFolder = pathToDataset + "/offline_img/";
	indexFile = pathToDataset + "/index.txt";
	objDimFile = pathToDataset + "/obj_dim_average.txt";
    
    ca::Profiler::enable(); // Activate tictoc_profiler time keeper

    // Define objects from other classes
    DatasetSunRGBD dataLoader; // Define a dataloader for the sun_rgbd dataset
    detect_cuboid_bbox objectDetector; // 3D cuboid detector class
    
    // Set configurations
    objectDetector.whether_plot_detail_images = true;
	objectDetector.whether_plot_ground_truth = false;
	objectDetector.whether_plot_sample_images = false;
	objectDetector.whether_plot_final_scores = true;
	objectDetector.whether_sample_obj_dimension = true;
	objectDetector.whether_sample_obj_yaw = true;
	objectDetector.whether_add_plane_constraints = true;
	objectDetector.whether_save_cam_obj_data = false;
	objectDetector.whether_save_final_image = false;

    objectDetector.Read_Dimension_SUNRGBD(objDimFile); // Table 1 from 2022 point-plane-object SLAM paper
    
    totalFrameNumber = vImageId.size();
    // totalFrameNumber = 50; 01/12/2024 As from benchun`s repo, I am not sure why

    // Initialization complete

}


SUNRGBDObjectDetector::~SUNRGBDObjectDetector()
{
    /*
    Class destructor, release resources and gracefully exit
    */
   pass;
}