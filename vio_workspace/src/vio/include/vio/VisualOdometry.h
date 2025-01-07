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
        void update_pose(cv::Mat& prev_mat, cv::Mat& curr_mat);

        // Call Destructor
        ~VisualOdometry();

    private:
        float feature_detection_prob = 0.75; // probability for "good" features to keep
        float ransac_prob = 0.9; // probability for RANSAC(outlier detection)
        // Create descriptors
        cv::Mat prev_descriptors, curr_descriptors;
        cv::FlannBasedMatcher flannMatcher; // Use the flann based matcher
        cv::Mat essentialMatrix, Rotation, Trans, triangulated_points;  // Initialize essential matrix, rotation and translation
        cv::Mat prev_R_and_T, curr_R_and_T;
        
};


#endif /* _VISUAL_ODOMETRY_H */