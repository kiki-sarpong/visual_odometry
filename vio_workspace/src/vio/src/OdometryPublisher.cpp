#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
// #include "tf2_msgs/msg/tf_message.hpp"
#include "std_msgs/msg/header.hpp"
#include "OdometryPublisher.h"



// Constructor
OdometryPublisher::OdometryPublisher() : Node("odometry_publisher"){
    // Set QOS profile for services and topics
    rclcpp::QoS qos_profile(qos_depth);
    qos_profile.reliable();  // Reliable communication
    qos_profile.durability_volatile();  // Retain last message
    std::string topic_name = "odometry_publisher";

    odometry_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>(topic_name, qos_profile);
    RCLCPP_INFO(this->get_logger(), (topic_name + " has started.").c_str());
}


/*
This method calls the visual odometry publisher.
It publishes a transform stamped msg, and gives every new message a unique
frame id based on a counter.
*/
 void OdometryPublisher::call_publisher(cv::Mat& Rotation, cv::Mat& Translation){
    // auto tf2_msg = tf2_msgs::msg::TFMessage(); // Create tf2_msg
    auto geo_msg = geometry_msgs::msg::PoseStamped(); // Create transform stamped msg
    geo_msg.header.stamp = this->get_clock()->now();
    geo_msg.header.frame_id = map;
    // // Make every child frame id unique 
    // geo_msg.child_frame_id = id_name + std::to_string(count);
    // geo_msg.transform.translation.x = Translation.at<double>(0);
    // geo_msg.transform.translation.y = Translation.at<double>(1);
    // geo_msg.transform.translation.z = Translation.at<double>(2);

    // tf2::Quaternion quaternion = OdometryPublisher::rot_to_quaternion(Rotation);
    // geo_msg.transform.rotation.x = quaternion.x();
    // geo_msg.transform.rotation.y = quaternion.y();
    // geo_msg.transform.rotation.z = quaternion.z();
    // geo_msg.transform.rotation.w = quaternion.w();

    geo_msg.pose.position.x = Translation.at<double>(0);
    geo_msg.pose.position.y = Translation.at<double>(1);
    geo_msg.pose.position.z = Translation.at<double>(2);

    tf2::Quaternion quaternion = OdometryPublisher::rot_to_quaternion(Rotation);
    geo_msg.pose.orientation.x = quaternion.x();
    geo_msg.pose.orientation.y = quaternion.y();
    geo_msg.pose.orientation.z = quaternion.z();
    geo_msg.pose.orientation.w = quaternion.w();

    // tf2_msg.transforms = geo_msg;

    // Publish message
    odometry_publisher->publish(geo_msg);

    count += 0.01;  // Increment count
   
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