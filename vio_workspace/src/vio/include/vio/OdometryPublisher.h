#ifndef _ODOMETRY_PUB_H
#define _ODOMETRY_PUB_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Matrix3x3.h"


class OdometryPublisher : public rclcpp::Node{
    public:
        OdometryPublisher();
        void call_publisher(cv::Mat& Rotation, cv::Mat& Translation);
        tf2::Quaternion rot_to_quaternion(const cv::Mat& rotation_matrix);

    private:
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr odometry_publisher;
        static const int qos_depth = 10;
        std::string map = "map";
        std::string id_name = "frame_";
        std::string topic_name;
        float count = 0.01;

};


#endif /* _ODOMETRY_PUB_H */