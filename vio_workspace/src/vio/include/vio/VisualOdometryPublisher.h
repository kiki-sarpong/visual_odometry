#ifndef _VISUAL_ODOMETRY_PUB_H
#define _VISUAL_ODOMETRY_PUB_H

#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"


class VisualOdometryPublisher : public rclcpp::Node {
    public:
        VisualOdometryPublisher();
        void call_publisher();

    private:
        rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr vo_publisher;
        static const int qos_depth = 10;

};


#endif /* _VISUAL_ODOMETRY_PUB_H */