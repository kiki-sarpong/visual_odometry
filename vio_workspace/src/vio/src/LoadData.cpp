
#include <eigen3/Eigen/Core>
#include <fstream>
#include <sstream>
#include <string>
#include <iostream>
#include "LoadData.h"

// Constructor
LoadData::LoadData(){};

// Destructor
LoadData::~LoadData(){};

/*
This is a method to load a 4 x 4 homogeous matrix
input: file_path
returns: A vector of matrices
*/
std::vector<Eigen::MatrixXd> LoadData::load_homogeneous_matrix(std::string& file_path, std::string name){
    // Create input-file stream object
    std::ifstream file(file_path);

    // Assign matrix size
    Eigen::MatrixXd homogeneous_matrix = Eigen::MatrixXd(row_4, col_4);

    // Create a vector to hold the 4 homogeneous matrices
    std::vector<Eigen::MatrixXd> matrix_vector; 
    
    // Check if file can be opened
    if(!file.is_open()){
        std::cerr <<  "File  "<<  file_path <<" could not be opened. Exiting ...\n";
        return matrix_vector;
    }
    
    // Read line
    std::string line;
    while(std::getline(file, line)){
        // std::cout << line << "\n";

        // Use a stringstream to parse the values
        std::stringstream ss(line);
        double element;
        int rw = 0, cl = 0;

        if(name == "calib"){
            // calib txt looks like P0: 7.188560000000e+02 0.000000000000e+00  ....
            // Skip the "P0:" label
            std::string label;
            ss >> label; 
        }

        // For every line, read the homogeneous matrix values on the line
        while(ss >> element){
            // Assign element to matrix location
            homogeneous_matrix(rw, cl) = element; 
            cl ++;
            if (cl == col_4){cl = 0; rw++;}  // Move to next row
        }
        // The last element of the matrix must be set to 1. Matrix is a homogeneous matrix
        homogeneous_matrix(row_4-1, col_4-1) = 1;

        matrix_vector.emplace_back(homogeneous_matrix);
        std::cout << homogeneous_matrix << "\n";
        std::cout << "------------------\n";

    }

    file.close();

    return matrix_vector;
}


/*
This is a method to load the timestamps from a file
input: file_path
returns: 1D vector of timestamps
*/
std::vector<double> LoadData::load_timestamps(std::string& file_path){
    // Load file into input-file stream object
    std::ifstream file(file_path);

    // Assign timestamp vector
    std::vector<double> timestamps;

    // Check if file can be opened
    if(!file.is_open()){
        std::cerr <<  "File  "<<  file_path <<" could not be opened. Exiting ...\n";
        return timestamps;
    }

    // Read line
    std::string line;
    while(std::getline(file, line)){
        // Use stringstream to parse line
        std::stringstream ss(line);
        double element;

        // For the timestamps, every line in the file has only one timestamp
        // Put the timestamp into a 1D vector
        while(ss >> element){timestamps.emplace_back(element);}
    }

    for(double& k:timestamps){
        std::cout << k << "\n";
    }
    return timestamps;
}