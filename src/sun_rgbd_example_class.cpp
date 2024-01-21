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

    RCLCPP_INFO(this->get_logger(), "\nPRESS ANY BUTTON TO CYCLE THROUGH IMAGES\n");


    // Build path to dataset
    std::string homeDir = getenv("HOME");
    pathToDataset = homeDir + "/" + "Documents/" + "SUN_RGBD_Selected"; //!HARDCODED see README.md file

    // // Declare command-line parameters
    // this->declare_parameter("path_to_sunrgbd_dataset_arg", "not_given"); // Path to SUN RGB-D dataset sample as used in benchun`s repository

    // // Populate path set by user from command-line
    // pathToDataset = "not_set";
    // rclcpp::Parameter param1 = this->get_parameter("path_to_sunrgbd_dataset_arg");
    // pathToDataset = param1.as_string();

    RCLCPP_INFO(this->get_logger(), "pathToDataset %s", pathToDataset.c_str());

    // Initialize paths
    imageFolder = pathToDataset + "/rgb/";
	depthFolder = pathToDataset + "/depth/";
	labelFolder = pathToDataset + "/label/";
	calibFolder = pathToDataset + "/calib/";
	outputFolder = pathToDataset + "/offline_cuboid/";
	outputImgFolder = pathToDataset + "/offline_img/";
	indexFile = pathToDataset + "/index.txt";
	objDimFile = pathToDataset + "/obj_dim_average.txt";
    
    // Load image index
	dataLoader.LoadImageIndex(indexFile);
	vImageId = dataLoader.vstrImageIndex;
    RCLCPP_INFO(this->get_logger(), "Number of image indicies: %zu", vImageId.size());
    

    //* Configurations to view certain outputs
    objectDetector.whether_plot_detail_images = false; // Shows normalized depth_map, edges detected by canny, edges inside the object
	objectDetector.whether_plot_ground_truth = false; //! 01/21/24 not used
	objectDetector.whether_plot_sample_images = false; // Show proposal of the 3d cuboids
	objectDetector.whether_plot_final_scores = true; // Shows the final selected image
	
    //* Benchun Zhou`s cuboid detection strategy from his 2022 point-plane-object SLAM paper
    objectDetector.whether_sample_obj_dimension = true;
	objectDetector.whether_sample_obj_yaw = true;
	objectDetector.whether_add_plane_constraints = true;

	//* Save results?
    objectDetector.whether_save_cam_obj_data = false;
	objectDetector.whether_save_final_image = false;

    objectDetector.Read_Dimension_SUNRGBD(objDimFile); // Table 1 from 2022 point-plane-object SLAM paper
    
    totalFrameNumber = vImageId.size();
    totalFrameNumber = 50; // Set a positive value < 400 if you want to test with a subset of the images

    // Initialization complete
}


SUNRGBDObjectDetector::~SUNRGBDObjectDetector()
{
    /*
    Class destructor, release resources and gracefully exit
    */
   cv::destroyAllWindows();
}

void SUNRGBDObjectDetector::detectCuboidsInOneImage(int frame_id){
    /*
       TODO 
    */

    
    // Initialize work variables
    std::string frameIndexString = ""; // Four digit index of the images
    frameIndexString = vImageId[frame_id]; 
    // RCLCPP_INFO(this->get_logger(), "Pulse1\n");
    
    // std::cout << "-----------" << "frameIndex " << frameIndex << " " << vImageId[frameIndex] << "-----------" << std::endl;
    RCLCPP_INFO(this->get_logger(), "frameIndexString: %s", frameIndexString.c_str());
    
    //read image, calib and input
    ca::Profiler::tictoc("read image");
    
    std::string imageFile = imageFolder + "/" + frameIndexString + ".jpg";
    std::string calibFile = calibFolder + "/" + frameIndexString + ".txt";
    std::string truthCuboidFile = labelFolder + "/" + frameIndexString + ".txt";

    // RCLCPP_INFO(this->get_logger(), "path to image file: %s", imageFile.c_str()); // Debug
    objectDetector.Read_Image_SUNRGBD(imageFile);
    objectDetector.Read_Kalib_SUNRGBD(calibFile);
    // objectDetector.Read_Label_SUNRGBD(truthCuboidFile); // Original
    objectDetector.Read_Label_SUNRGBD(truthCuboidFile, false);
    
    ca::Profiler::tictoc("read image");

    // read depth image
    ca::Profiler::tictoc("planes from depth");

    string depthImgFile = depthFolder + "/" + frameIndexString + ".png";
    planeDetector.setDepthValue(8000); //? why 8000?
    planeDetector.setKalibValue(objectDetector.Kalib);
    planeDetector.readDepthImage(depthImgFile); // 01/18/24 fixed segmentation fault
    planeDetector.ConvertDepthToPointCloud();
    planeDetector.ComputePlanesFromOrganizedPointCloud(); // Error here 01/18/24, fixed 01/24/2024
    
    ca::Profiler::tictoc("planes from depth");
    
    // RCLCPP_INFO(this->get_logger(), "Pulse1\n");
    // RCLCPP_INFO(this->get_logger(), "Pulse2\n");

    ca::Profiler::tictoc("cuboid detection");
    std::vector<ObjectSet> framesCuboids; // each 2d bbox generates an ObjectSet, which is vector of sorted proposals
    cv::Mat rgbImg = objectDetector.rgb_img.clone();
    std::vector<cv::Mat> detPlane = planeDetector.mvPlaneCoefficients;
    
    // create save file for every frame
    string outputCuboidFile = outputFolder + "/" + frameIndexString + "_3d_cuboids.txt";
    string outputCuboidImg = outputImgFolder + "/" + frameIndexString + "_3d_img.png";
    objectDetector.detect_cuboid_every_frame(rgbImg, detPlane, framesCuboids, outputCuboidFile, outputCuboidImg);
}