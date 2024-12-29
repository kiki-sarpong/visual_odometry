#ifndef _LOAD_DATA_H
#define _LOAD_DATA_H

#include <eigen3/Eigen/Core>
#include <fstream>
#include <sstream>
#include <string>


class LoadData{
    public:
        // Call constructor
        LoadData();

        // Call Destructor
        ~LoadData();

        std::vector<Eigen::MatrixXd> load_homogeneous_matrix(std::string& file_path, std::string name = "none");
        std::vector<double> load_timestamps(std::string& file_path);

    private:
        int row_4 = 4;
        int col_4 = 4;

};

#endif /* _LOAD_DATA_H */