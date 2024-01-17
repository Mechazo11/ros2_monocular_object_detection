#ifndef SUN_RGBD_H
#define SUN_RGBD_H

// std c
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Core>

using namespace std;

class DatasetSunRGBD
{
public:
    /* data */
    vector<string> vstrImageIndex;
    std::vector<std::string> obj_name_list;// label: class(1), count(1), dim(3), 
    Eigen::MatrixXd obj_dim_list; 

public:
    DatasetSunRGBD(/* args */); // Constructor
    ~DatasetSunRGBD(); // Destructor
    void LoadImageIndex(const string &strFile); // For a given image, find its index
    int LoadObjectGT(const string &obj_dim_file); // Originally void LoadObjectGT(const string &obj_dim_file);
};

#endif