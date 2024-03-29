#ifndef DETECT_CUBOID_BBOX_H
#define DETECT_CUBOID_BBOX_H

// std c
#include <string>

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/core/core.hpp>

#include "ros2_monocular_object_detection/detect_cuboid_bbox/matrix_utils.h"

/*
class cuboid // matlab cuboid struct. cuboid on ground. only has yaw, no obj roll/pitch
{
    public:
      Eigen::Vector3d pos;
      Eigen::Vector3d scale;
      double rotY;

      Eigen::Vector2d box_config_type;       // configurations, vp1 left/right
      Eigen::Matrix2Xi box_corners_2d;       // 2*8
      Eigen::Matrix3Xd box_corners_3d_world; // 3*8

      Eigen::Vector4d rect_detect_2d; //% 2D bounding box (might be expanded by me)
      double edge_distance_error;
      double edge_angle_error;
      double normalized_error; // normalized distance+angle
      double skew_ratio;
      double down_expand_height;
      double camera_roll_delta;
      double camera_pitch_delta;

      void print_cuboid(); // print pose information
};
typedef std::vector<cuboid *> ObjectSet; // for each 2D box, the set of generated 3D cuboids

struct cam_pose_infos
{
      Eigen::Matrix4d transToWolrd;
      Eigen::Matrix3d Kalib;

      Eigen::Matrix3d rotationToWorld;
      Eigen::Vector3d euler_angle;
      Eigen::Matrix3d invR;
      Eigen::Matrix3d invK;
      Eigen::Matrix<double, 3, 4> projectionMatrix;
      Eigen::Matrix3d KinvR; // K*invR
      double camera_yaw;
};


class detect_3d_cuboid
{
    public:
      cam_pose_infos cam_pose;
      cam_pose_infos cam_pose_raw;
      void set_calibration(const Eigen::Matrix3d &Kalib);
      void set_cam_pose(const Eigen::Matrix4d &transToWolrd);

      // object detector needs image, camera pose, and 2D bounding boxes(n*5, each row: xywh+prob)  long edges: n*4.  all number start from 0
      void detect_cuboid(const cv::Mat &rgb_img, const Eigen::Matrix4d &transToWolrd, const Eigen::MatrixXd &obj_bbox_coors, Eigen::MatrixXd edges,
                         std::vector<ObjectSet> &all_object_cuboids);
      void detect_cuboid_new(cv::Mat &rgb_img, Eigen::Matrix4d &transToWolrd, const Eigen::MatrixXd &obj_bbox_coors, Eigen::MatrixXd edges,
                         std::vector<ObjectSet> &all_object_cuboids);

      bool whether_plot_detail_images = false;
      bool whether_plot_final_images = false;
      bool whether_save_final_images = false;
      cv::Mat cuboids_2d_img; // save to this opencv mat
      bool print_details = false;

      // important mode parameters for proposal generation.
      bool consider_config_1 = true; // false true
      bool consider_config_2 = true;
      bool whether_sample_cam_roll_pitch = false; // sample camera roll pitch in case don't have good camera pose
      bool whether_sample_bbox_height = false;    // sample object height as raw detection might not be accurate

      int max_cuboid_num = 1;        //final return best N cuboids
      double nominal_skew_ratio = 1; // normally this 1, unless there is priors
      double max_cut_skew = 3;
};

*/


class cuboid
{
public:
    std::string obj_name;
    Eigen::Vector4d bbox_2d; //% 2D bounding box 

    Eigen::Vector3d pos;   // global value
    Eigen::Vector3d scale;    // global value
    double rotY;              // global value

    Eigen::Matrix3d  obj_rot_camera;
    Eigen::Vector3d  obj_loc_camera;
    Eigen::Vector3d  obj_dim_camera;
    double edge_distance_error;
    double edge_angle_error;
    double plane_obj_error;
    double overall_error;

    Eigen::Matrix2Xd box_corners_2d;       // 2*8
    Eigen::Matrix3Xd box_corners_3d_world; // 3*8
    Eigen::Matrix3Xd box_corners_3d_cam; // 3*8

    void print_cuboid(); // print pose information
};
typedef std::vector<cuboid *> ObjectSet; // for each 2D box, the set of generated 3D cuboids

//* Main class
class detect_cuboid_bbox
{
public:
    //* Parameters
    bool whether_plot_detail_images = false;
    bool whether_plot_ground_truth = false;
    bool whether_plot_sample_images = false;
    bool whether_plot_final_scores = false;
    bool whether_sample_obj_dimension = true;
    bool whether_sample_obj_yaw = true;
    bool whether_add_plane_constraints = true;
    bool whether_save_cam_obj_data = true;
    bool whether_save_final_image = true;

    //* Work variables
    cv::Mat rgb_img;
    Eigen::Matrix4d Twc; // transToWolrd
    Eigen::Matrix3d Kalib;

    std::vector<std::string> dataset_obj_name_list;// dataset object label: class(1), count(1), dim(3), 
    Eigen::MatrixXd dataset_obj_dim_list; 
	std::vector<std::string> detected_obj_name;// frame object label: class(1), 2d bbox(4), centroid(3), dim(3), orient(2)
	Eigen::MatrixXd detected_obj_input; 

    int waitKeyval = 1000; // Any positive integer, milliseconds, if 0, window waits until a key is pressed

public:
    //* Function definitions
    bool Read_Image_SUNRGBD(std::string & img_file); 
    bool Read_Kalib_SUNRGBD(std::string &calib_file);
    bool Read_Dimension_SUNRGBD(std::string &dim_file);
    // bool Read_Label_SUNRGBD(std::string &label_file); // Original
    bool Read_Label_SUNRGBD(std::string &label_file, bool showTruthCuboidList);
    bool Get_Object_Input_SUNRGBD(const int& index, std::string &obj_name, 
            Eigen::Vector4d &obj_bbox, Eigen::Vector3d& obj_dim, double& obj_yaw);
    void detect_cuboid_every_frame(cv::Mat& rgb_img, std::vector<cv::Mat>& mvPlaneCoefficients, 
		std::vector<ObjectSet>& frame_cuboid, std::string& output_file, std::string& output_img);
    void detect_cuboid_with_bbox_constraints(cv::Mat& rgb_img, Eigen::Vector4d& obj_bbox, double& obj_yaw,
            const Eigen::Vector3d& obj_dim_ave, std::vector<cv::Mat>& mvPlaneCoefficients, std::vector<cuboid *>& single_object_candidate);
    void compute_obj_visible_edge(cuboid* new_cuboid, 
        Eigen::MatrixXi& visible_edge_pt_ids, Eigen::MatrixXd& box_edges_visible);
    void formulate_cuboid_param(cuboid* new_cuboid, Eigen::Vector3d& obj_loc_cam, 
            Eigen::Matrix3d& obj_rot_cam, Eigen::Vector3d& obj_dim_cam, Eigen::Matrix3d& Kalib);
};


#endif // DETECT_CUBOID_BBOX_H