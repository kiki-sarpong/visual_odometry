#ifndef _GROUND_TRUTH_PUB_H
#define _GROUND_TRUTH_PUB_H

#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"


class GroundTruthPublisher : public rclcpp::Node {
    public:
        GroundTruthPublisher();
        void call_publisher();

    private:
        rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr gt_publisher;
        static const int qos_depth = 10;

};


#endif /* _GROUND_TRUTH_PUB_H */