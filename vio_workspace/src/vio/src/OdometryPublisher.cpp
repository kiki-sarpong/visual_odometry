#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/header.hpp"
#include "OdometryPublisher.h"
#include "VisualOdometry.h"


// Constructor
OdometryPublisher::OdometryPublisher(VisualOdometry* main_node_, std::string tp_name) : main_node(main_node_), topic_name(tp_name){
    // Set QOS profile for services and topics
    rclcpp::QoS qos_profile(qos_depth);
    qos_profile.reliable();  // Reliable communication
    qos_profile.durability_volatile();  // Retain last message

    odometry_publisher = main_node->create_publisher<geometry_msgs::msg::PoseArray>(topic_name, qos_profile);
    RCLCPP_INFO(main_node->get_logger(), (topic_name + " has started.").c_str());
}


/*
This method calls the visual odometry publisher.
*/
 void OdometryPublisher::call_publisher(cv::Mat& matrix_RT, std::string pub_type){
    auto pose_array_msg = geometry_msgs::msg::PoseArray(); 
    pose_array_msg.header.stamp = main_node->get_clock()->now();
    pose_array_msg.header.frame_id = map;

    // Get Rotation and Translation from matrix_RT
    cv::Mat Rotation = matrix_RT(cv::Range(0, 3), cv::Range(0, 3));
    cv::Mat Translation = matrix_RT(cv::Range(0, 3), cv::Range(3, 4));

    auto p_msg = geometry_msgs::msg::Pose();
    if(pub_type=="ground_truth"){
        p_msg.position.x = Translation.at<double>(0);
        p_msg.position.y = Translation.at<double>(2);
        p_msg.position.z = 0.0; //Translation.at<double>(1);
    }
    else{
        p_msg.position.x = Translation.at<double>(0);
        p_msg.position.y = Translation.at<double>(2);
        p_msg.position.z = 0.0; // Translation.at<double>(1);
    }
    

    tf2::Quaternion quaternion = OdometryPublisher::rot_to_quaternion(Rotation);
    p_msg.orientation.x = quaternion.x();
    p_msg.orientation.y = quaternion.y();
    p_msg.orientation.z = quaternion.z();
    p_msg.orientation.w = quaternion.w();

    pose_vector.emplace_back(p_msg);

    pose_array_msg.poses = pose_vector;
    // Publish message
    odometry_publisher->publish(pose_array_msg);
    // RCLCPP_INFO(this->get_logger(), std::to_string(pose_vector.size()).c_str());
    // std::cout <<p_msg.position.x << "   " << p_msg.position.y << " \n";
    // std::cout << matrix_RT  << " \n";   
}


/*
This method converts a cv::Mat to a quaternion
input: rotation matrix
returns: a quaternion
*/
tf2::Quaternion OdometryPublisher::rot_to_quaternion(const cv::Mat& rotation_matrix) {
    tf2::Matrix3x3 tf2_rot(rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1), rotation_matrix.at<double>(0, 2),
                            rotation_matrix.at<double>(1, 0), rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
                            rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1), rotation_matrix.at<double>(2, 2));

    tf2::Quaternion quaternion;
    tf2_rot.getRotation(quaternion); // Extract quartenion from rotation

    return quaternion;
}