#include <iostream>
#include <string>
#include <eigen3/Eigen/Core>
#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include "LoadData.h"
#include "GroundTruthPublisher.h"
#include "VisualOdometryPublisher.h"
#include "VisualOdometry.h"

// Constructor
VisualOdometry::VisualOdometry() : Node("Visual_odometry"){
    run_visual_odometry();
};

// Destructor
VisualOdometry::~VisualOdometry(){};


/*
This method runs the main visual odometry pipeline
input: void
returns: void
*/
void VisualOdometry::run_visual_odometry(){
    std::string dataset_num = "00";
    std::string dataset_file_path = "/home/vio_dev/vio_workspace/src/dataset/sequences/" + dataset_num + "/";
    std::string calibration_file = dataset_file_path + "calib.txt";
    std::string timestamp_file = dataset_file_path + "times.txt";
    // Set path for directories with image data
    std::string left_images_dir = dataset_file_path + "image_0/";
    std::string right_images_dir = dataset_file_path + "image_1/";
    // Ground truth dataset ranges from 00 - 10.txt
    std::string ground_truth_path = "/home/vio_dev/vio_workspace/src/poses_dataset/poses/" + dataset_num + ".txt";

    LoadData data;
    std::vector<Eigen::MatrixXd> calib_data = data.load_homogeneous_matrix(calibration_file, "calib");
    RCLCPP_INFO(this->get_logger(), "Loading calibration matrix data ....");

    std::vector<Eigen::MatrixXd> ground_truth = data.load_homogeneous_matrix(ground_truth_path);
    RCLCPP_INFO(this->get_logger(), "Loading ground truth data ....");

    std::vector<double> timestamps = data.load_timestamps(timestamp_file);
    RCLCPP_INFO(this->get_logger(), "Loading timestamp data ....");

    int number_of_images = 100;
    std::vector<std::string> left_images = data.load_images(left_images_dir, number_of_images);

    if (number_of_images != -1){
        RCLCPP_INFO(this->get_logger(), ("loading " + std::to_string(number_of_images)  + " images for visual odometry").c_str());
    }
    else{
        RCLCPP_INFO(this->get_logger(), ("loading " + std::to_string(timestamps.size()) + " images for visual odometry").c_str());
    }
    


}