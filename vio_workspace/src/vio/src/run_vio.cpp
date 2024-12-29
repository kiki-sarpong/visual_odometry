#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <eigen3/Eigen/Core>
#include <opencv2/opencv.hpp>
#include "LoadData.h"


int main() {

    std::string dataset_num = "00";
    std::string dataset_file_path = "/home/vio_dev/vio_workspace/src/dataset/sequences/" + dataset_num + "/";
    std::string calibration_file = dataset_file_path + "calib.txt";
    std::string timestamp_file = dataset_file_path + "times.txt";
    // Ground truth dataset ranges from 00 - 10.txt
    std::string ground_truth_path = "/home/vio_dev/vio_workspace/src/poses_dataset/poses/" + dataset_num + ".txt";

    LoadData data;
    // std::vector<Eigen::MatrixXd> calib_data = data.load_homogeneous_matrix(calibration_file, "calib");

    // std::vector<Eigen::MatrixXd> ground_truth = data.load_homogeneous_matrix(ground_truth_path);

    std::vector<double> timestamps = data.load_timestamps(timestamp_file);


    return 0;
}
