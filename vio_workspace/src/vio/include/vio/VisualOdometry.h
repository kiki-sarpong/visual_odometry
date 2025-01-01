#ifndef _VISUAL_ODOMETRY_H
#define _VISUAL_ODOMETRY_H

#include <iostream>
#include "rclcpp/rclcpp.hpp"

class VisualOdometry : public rclcpp::Node {
    public:
        // call Constructor
        VisualOdometry();
        void run_visual_odometry();

        // call Destructor
        ~VisualOdometry();

    private:

};


#endif /* _VISUAL_ODOMETRY_H */