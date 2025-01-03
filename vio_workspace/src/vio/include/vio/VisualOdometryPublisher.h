#ifndef _VISUAL_ODOMETRY_PUB_H
#define _VISUAL_ODOMETRY_PUB_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Matrix3x3.h"


class VisualOdometryPublisher : public rclcpp::Node {
    public:
        VisualOdometryPublisher();
        void call_publisher(cv::Mat& Rotation, cv::Mat& Translation);
        tf2::Quaternion rot_to_quaternion(const cv::Mat& rotation_matrix);

    private:
        rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr vo_publisher;
        static const int qos_depth = 10;
        std::string base_link = "base_link";
        std::string id_name = "frame_";
        float count = 0.01;

};


#endif /* _VISUAL_ODOMETRY_PUB_H */