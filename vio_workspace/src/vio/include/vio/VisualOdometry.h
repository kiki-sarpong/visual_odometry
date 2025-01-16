#ifndef _VISUAL_ODOMETRY_H
#define _VISUAL_ODOMETRY_H

#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include <eigen3/Eigen/Core>
#include <opencv2/opencv.hpp>


class VisualOdometry : public rclcpp::Node {
    public:
        // Call Constructor
        VisualOdometry();
        void run_visual_odometry();
        cv::Mat decompose_matrix(cv::Mat& projection_matrix);
        cv::Mat eigen_to_cv(Eigen::MatrixXd& eigen_matrix);
        cv::Mat create_R_and_T_matrix(cv::Mat& rotation, cv::Mat& translation);
        double get_scale(std::vector<Eigen::MatrixXd>& eigen_matrix_vector, const int idx);
        void detect_features(cv::Mat& image, std::vector<cv::Point2f>& points);

        // Call Destructor
        ~VisualOdometry();

    private:
        int minimum_feature_count = 1000; // Threshold for minimum number of features
        float ransac_prob = 0.85; // Probability for RANSAC(outlier detection)
        int fast_threshold = 20;    // Threshold for FAST algorithm
        bool nonmaxSuppression = true;
        cv::Mat essentialMatrix, Rotation, Trans, triangulated_points;  // Initialize essential matrix, rotation and translation
        cv::Mat prev_R_and_T, curr_R_and_T;
        
};


#endif /* _VISUAL_ODOMETRY_H */