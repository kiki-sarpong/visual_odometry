#include "std_msgs/msg/header.hpp"
#include "PointCloudPublisher.h"
#include "VisualOdometry.h"


// Constructor
PointCloudPublisher::PointCloudPublisher(VisualOdometry* main_node_, std::string tp_name) : main_node(main_node_), topic_name(tp_name){
    // Set QOS profile for services and topics
    rclcpp::QoS qos_profile(qos_depth);
    qos_profile.reliable();  // Reliable communication
    qos_profile.durability_volatile();  // Retain last message

    pointcloud_publisher = main_node->create_publisher<sensor_msgs::msg::PointCloud>(topic_name, qos_profile);
    RCLCPP_INFO(main_node->get_logger(), (topic_name + " has started.").c_str());
}


/*
This method calls the visual odometry publisher.
input: a vector of 3d observations
output: void
*/
 void PointCloudPublisher::call_publisher(std::vector<Eigen::MatrixXd>& observations_3d){
    auto point_cloud_msg = sensor_msgs::msg::PointCloud(); 
    point_cloud_msg.header.stamp = main_node->get_clock()->now();
    point_cloud_msg.header.frame_id = map;

    // Assign all the observation points to a pointcloud
    for (int matrix=0; matrix<observations_3d.size(); matrix++){
        for (int i=0; i<observations_3d[matrix].rows(); i++){
            geometry_msgs::msg::Point32 pt_32 = geometry_msgs::msg::Point32();
            pt_32.x = observations_3d[matrix](i,0);
            pt_32.y = observations_3d[matrix](i,1);
            pt_32.z = observations_3d[matrix](i,2);
            pointcloud_points.emplace_back(pt_32);
        }
    }

    // Assign array of point32 points 
    point_cloud_msg.points = pointcloud_points;
    // Publish message
    pointcloud_publisher->publish(point_cloud_msg);

}
