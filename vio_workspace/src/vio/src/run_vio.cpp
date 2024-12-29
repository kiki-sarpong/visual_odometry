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
    std::string ground_truth = "/home/vio_dev/vio_workspace/src/poses_dataset/poses/" + dataset_num + ".txt";

    LoadData data;
    std::string calib_name = "calib";
    std::vector<Eigen::MatrixXd> calib_data = data.load_homogeneous_matrix(calibration_file, calib_name);

    // // Create input stream file object
    // std::ifstream calib_file(calibration_file);

    // if(!calib_file.is_open()){
    //     std::cerr << "Calibration file could not be opened.\n";
    //     return -1;
    // }

    // int row = 4, col = 4;
    // // Create a row by col matrix for the calibration values
    // // Eigen::matrix<double, row, col> calib_matrix;

    // // Add layer to create homogeneous matrix
    // // Eigen::matrixXd add_layer = MatrixXd(cre);

    // Eigen::MatrixXd calib_matrix = Eigen::MatrixXd(row, col);

    // // Create a vector to hold the 4 projection matrices
    // std::vector<Eigen::MatrixXd> projection_matrix;

    // // Read line
    // std::string line;
    // while(std::getline(calib_file,line)){
    //     // std::cout << line << "\n";

    //     // Use a stringstream to parse the values
    //     std::stringstream ss(line);
    //     double element;
    //     int rw = 0, cl = 0;

    //     // string looks like P0: 7.188560000000e+02 0.000000000000e+00  ....
    //     // Skip the "P0:" label
    //     std::string label;
    //     ss >> label; 

    //     // For every line, read the projection matrix values on the line
    //     while(ss >> element){
    //         // Assign element to matrix location
    //         calib_matrix(rw, cl) = element; 
    //         cl ++;
    //         if (cl == col){cl = 0; rw++;}  // Move to next row
    //     }
    //     // The last element of the matrix must be set to 1. Matrix is a homogeneous matrix
    //     calib_matrix(row-1, col-1) = 1;

    //     projection_matrix.emplace_back(calib_matrix);
    //     std::cout << calib_matrix << "\n";
    //     std::cout << "------------------\n";

    // }

    // calib_file.close();

    return 0;
}
