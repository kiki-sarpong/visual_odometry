#include <iostream>
#include <string>
// #include <eigen3/Eigen/Core>
// #include <opencv2/opencv.hpp>
// #include "LoadData.h"
#include "rclcpp/rclcpp.hpp"
#include "GroundTruthPublisher.h"
#include "VisualOdometryPublisher.h"


int main(int argc, char** argv) {
    // Initialize ROS2 node
    rclcpp::init(argc, argv);
    // // Keep node active and create publisher
    // rclcpp::spin(std::make_shared<VisualOdometryPublisher>());

    // Create seperate nodes for publishing visual odometry and ground truth
    auto visual_odom_publisher = std::make_shared<VisualOdometryPublisher>();
    auto ground_truth_publisher = std::make_shared<GroundTruthPublisher>();

    // Create multithreaded executor
    rclcpp::executors::MultiThreadedExecutor executor;

    // Add nodes to the executor
    executor.add_node(visual_odom_publisher);
    executor.add_node(ground_truth_publisher);

    // Spin the executor
    executor.spin();

    // Shutdown ROS2 nodde
    rclcpp::shutdown();
    return 0;

    // std::string dataset_num = "00";
    // std::string dataset_file_path = "/home/vio_dev/vio_workspace/src/dataset/sequences/" + dataset_num + "/";
    // std::string calibration_file = dataset_file_path + "calib.txt";
    // std::string timestamp_file = dataset_file_path + "times.txt";
    // // Set path for directories with image data
    // std::string left_images_dir = dataset_file_path + "image_0/";
    // std::string right_images_dir = dataset_file_path + "image_1/";
    // // Ground truth dataset ranges from 00 - 10.txt
    // std::string ground_truth_path = "/home/vio_dev/vio_workspace/src/poses_dataset/poses/" + dataset_num + ".txt";

    // LoadData data;
    // // std::vector<Eigen::MatrixXd> calib_data = data.load_homogeneous_matrix(calibration_file, "calib");

    // // std::vector<Eigen::MatrixXd> ground_truth = data.load_homogeneous_matrix(ground_truth_path);

    // // std::vector<double> timestamps = data.load_timestamps(timestamp_file);

    // std::vector<std::string> left_images = data.load_images(left_images_dir, 100);

}
