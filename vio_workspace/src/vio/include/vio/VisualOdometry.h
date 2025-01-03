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

        // Call Destructor
        ~VisualOdometry();

    private:
        float feature_detection_prob = 0.5; // probability for "good" features to keep
        float ransac_prob = 0.7; // probability for RANSAC(outlier detection)
        // Create descriptors
        cv::Mat prev_descriptors, curr_descriptors;
        std::vector<cv::DMatch> matches;  // Get matches
        std::vector<cv::DMatch> good_matches;  // Get good matches

        // Create keypoints
        std::vector<cv::KeyPoint> prev_keypoints, curr_keypoints;
        cv::FlannBasedMatcher flannMatcher; // Use the flann based matcher
        // Create prev_q and curr_q using the good matches
        std::vector<cv::Point2f> prev_q, curr_q;
        cv::Mat essentialMatrix, Rotation, Trans, triangulated_points;  // Initialize essential matrix, rotation and translation
        cv::Mat prev_R_and_T, curr_R_and_T;
        std::vector<uchar> mask;  // Initialize mask

};


#endif /* _VISUAL_ODOMETRY_H */