#ifndef _ODOMETRY_PUB_H
#define _ODOMETRY_PUB_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Matrix3x3.h"


class OdometryPublisher : public rclcpp::Node{
    public:
        OdometryPublisher();
        void call_publisher(cv::Mat& matrix_RT);
        tf2::Quaternion rot_to_quaternion(const cv::Mat& rotation_matrix);

    private:
        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr odometry_publisher;
        std::vector<geometry_msgs::msg::Pose> pose_vector;
        static const int qos_depth = 10;
        std::string map = "map";
        std::string id_name = "frame_";
        std::string topic_name;
        float count = 1.0;

};


#endif /* _ODOMETRY_PUB_H */