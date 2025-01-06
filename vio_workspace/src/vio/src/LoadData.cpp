#include <fstream>
#include <sstream>
#include <string>
#include <iostream>
#include <filesystem>
#include "LoadData.h"


// Constructor
LoadData::LoadData(){};

// Destructor
LoadData::~LoadData(){};

/*
This is a method to load a 3 x 4 matrix
input: file_path
returns: A vector of matrices
*/
std::vector<Eigen::MatrixXd> LoadData::load_matrix(std::string& file_path, std::string name){
    // Create input-file stream object
    std::ifstream file(file_path);

    Eigen::MatrixXd matrix_(row, col);

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
            // calib txt looks like "P0: 7.188560000000e+02 0.000000000000e+00  ...."
            // Do this to skip the "P0:" label
            std::string label;
            ss >> label; 
        }

        // For every line, read the homogeneous matrix values on the line
        while(ss >> element){
            // Assign element to matrix location
            matrix_(rw, cl) = element; 
            cl ++;
            if (cl == col){cl = 0; rw++;}  // Move to next row
        }

        matrix_vector.emplace_back(matrix_);
        // std::cout << matrix_ << "\n";
        // std::cout << "------------------\n";

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

    // for(double& k:timestamps){
    //     std::cout << k << "\n";
    // }
    return timestamps;
}


/*
This is a method to load the image file names into a 1D array
input: file_path (image_directory_path), number_to_load (-1 means load all pictures in folder)
returns: 1D vector of image file destinations
Note: The directory should NOT contain any other filetypes other than PNG images.
*/
std::vector<std::string> LoadData::load_images(std::string& file_path, int number_to_load){
    std::vector<std::string> vector_of_image_paths;

    if (!std::filesystem::exists(file_path) && !std::filesystem::is_directory(file_path)){
        std::cerr << "File " << file_path << " doesn't exist or isn't a directory";
        return vector_of_image_paths;
    }

    int file_count = 0;
    int prefix = 1000000;
    std::string post_fix = ".png"; // Image is a png file
    std::string name;

    // Iterate through the files in the directory
    for(const auto& file: std::filesystem::directory_iterator(file_path)){

        if (!std::filesystem::is_regular_file(file)) {  // Only count regular files (ignore directories)
            std::cerr << "File type error. Skipping...";
        }

        // If number_to_load variable is set, return vector
        if(number_to_load != -1 && number_to_load == file_count){
            return vector_of_image_paths;
        }

        // Image names are 000000.png , 0000001.png .... 000100.png, etc
        name = std::to_string(prefix+file_count);
        // Remove the '1' at the beginning of the number. 1000001 = 000001
        name = file_path + name.substr(1,name.length() - 1) + post_fix;  
        // std::cout << name << "\n";

        vector_of_image_paths.emplace_back(name);

        file_count +=1;  // Increment file_count
    }

    return vector_of_image_paths;
}