#ifndef _POINTCLOUD_PUBLISHER_H
#define _POINTCLOUD_PUBLISHER_H

#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "VisualOdometry.h"


class PointCloudPublisher{
    public:
        // Pass main node into publisher
        PointCloudPublisher(VisualOdometry* main_node_, std::string tp_name);
        void call_publisher(Eigen::MatrixXd& observations_3d);
        
    private:
        // Create publisher
        rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pointcloud_publisher;
        static const int qos_depth = 10;
        std::string map = "map";
        std::string topic_name;
        VisualOdometry* main_node;
        // Saves all point cloud points
        std::vector<geometry_msgs::msg::Point32> pointcloud_points;
};


#endif /* _POINTCLOUD_PUBLISHER_H */