// TODO need one line descriptor

// Includes
#include "ros2_monocular_object_detection/sun_rgbd_example_class.hpp"

SUNRGBDObjectDetector::SUNRGBDObjectDetector() :Node("point_plane_object_cuboid")
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

    // Build path to dataset
    std::string homeDir = getenv("HOME");
    pathToDataset = homeDir + "/" + "Documents/" + "SUN_RGBD_Selected"; //!HARDCODED see README.md file

    // // Declare command-line parameters
    // this->declare_parameter("path_to_sunrgbd_dataset_arg", "not_given"); // Path to SUN RGB-D dataset sample as used in benchun`s repository

    // // Populate path set by user from command-line
    // pathToDataset = "not_set";
    // rclcpp::Parameter param1 = this->get_parameter("path_to_sunrgbd_dataset_arg");
    // pathToDataset = param1.as_string();

    // RCLCPP_INFO(this->get_logger(), "pathToDataset %s", pathToDataset.c_str());

    imageFolder = pathToDataset + "/rgb/";
	depthFolder = pathToDataset + "/depth/";
	labelFolder = pathToDataset + "/label/";
	calibFolder = pathToDataset + "/calib/";
	outputFolder = pathToDataset + "/offline_cuboid/";
	outputImgFolder = pathToDataset + "/offline_img/";
	indexFile = pathToDataset + "/index.txt";
	objDimFile = pathToDataset + "/obj_dim_average.txt";
    
    ca::Profiler::enable(); // Activate tictoc_profiler time keeper

    

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
    totalFrameNumber = 50; // 01/12/2024 As is from benchun`s repo, I am not sure why

    // Initialization complete

}


SUNRGBDObjectDetector::~SUNRGBDObjectDetector()
{
    /*
    Class destructor, release resources and gracefully exit
    */
   pass;
}

void SUNRGBDObjectDetector::detectCuboidsInOneImage(int frameIndex){
    /*
       TODO 
    */

    std::cout << "-----------" << "frameIndex " << frameIndex << " " << vImageId[frameIndex] << "-----------" << std::endl;
    
    //read image, calib and input
    ca::Profiler::tictoc("Read image, calibration and gt files");
    
    std::string imageFile = imageFolder + "/" + vImageId[frameIndex] + ".jpg";
    objectDetector.Read_Image_SUNRGBD(imageFile);
    std::string calibFile = calibFolder + "/" + vImageId[frameIndex] + ".txt";
    objectDetector.Read_Kalib_SUNRGBD(calibFile);
    std::string truthCuboidFile = labelFolder + "/" + vImageId[frameIndex] + ".txt";
    objectDetector.Read_Label_SUNRGBD(truthCuboidFile);
    
    ca::Profiler::tictoc("Read image, calibration and gt files");

    // read depth image
    ca::Profiler::tictoc("Detect plane from depth image");

    string depthImgFile = depthFolder + "/" + vImageId[frameIndex] + ".png";
    planeDetector.setDepthValue(8000); //? why 8000?
    planeDetector.setKalibValue(objectDetector.Kalib);
    planeDetector.readDepthImage(depthImgFile);
    planeDetector.ConvertDepthToPointCloud();
    planeDetector.ComputePlanesFromOrganizedPointCloud();
    
    ca::Profiler::tictoc("Detect plane from depth image");

    std::vector<ObjectSet> framesCuboids; // each 2d bbox generates an ObjectSet, which is vector of sorted proposals
    cv::Mat rgbImg = objectDetector.rgb_img.clone();
    std::vector<cv::Mat> detPlane = planeDetector.mvPlaneCoefficients;
    
    // create save file for every frame
    string outputCuboidFile = outputFolder + "/" + vImageId[frameIndex] + "_3d_cuboids.txt";
    string outputCuboidImg = outputImgFolder + "/" + vImageId[frameIndex] + "_3d_img.png";
    objectDetector.detect_cuboid_every_frame(rgbImg, detPlane, framesCuboids, outputCuboidFile, outputCuboidImg);
}