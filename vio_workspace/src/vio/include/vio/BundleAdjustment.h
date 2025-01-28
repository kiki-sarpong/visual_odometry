#ifndef _BUNDLE_ADJUSTMENT_H
#define _BUNDLE_ADJUSTMENT_H

#include <ceres/ceres.h>
#include <ceres/rotation.h>
// #include <sophus/se3.hpp>
#include <opencv2/opencv.hpp>


class BundleAdjustment{
    public:
        // Constructor
        BundleAdjustment(double observed_x_, double observed_y_) : observed_x(observed_x_), \
         observed_y(observed_y_){}
        
        template <typename T>
        bool operator()(const T* const camera,
                        const T* const point,
                        T* residuals) const {
            // camera[0,1,2] are the angle-axis rotation.
            T p[3];
            ceres::AngleAxisRotatePoint(camera, point, p);
            // camera[3,4,5] are the translation.
            p[0] += camera[3]; p[1] += camera[4]; p[2] += camera[5];

            // Compute the center of distortion. The sign change comes from
            // the camera model that Noah Snavely's Bundler assumes, whereby
            // the camera coordinate system has a negative z axis.
            T xp = - p[0] / p[2];
            T yp = - p[1] / p[2];

            // Apply second and fourth order radial distortion.
            const T& l1 = camera[7];
            const T& l2 = camera[8];
            T r2 = xp*xp + yp*yp;
            T distortion = 1.0 + r2  * (l1 + l2  * r2);

            // Compute final projected point position.
            const T& focal = camera[6];
            T predicted_x = focal * distortion * xp;
            T predicted_y = focal * distortion * yp;

            // The error is the difference between the predicted and observed position.
            residuals[0] = predicted_x - T(observed_x);
            residuals[1] = predicted_y - T(observed_y);
            return true;
        }

        ~BundleAdjustment(){}; // Destructor

    private:
        double observed_x, observed_y;
        Eigen::Matrix3d intrinsics;
};


void run_bundle_adjustment(std::vector<std::vector<cv::Point2f>>& observations_2d, std::vector<Eigen::Vector3d>& observations_3d,
 std::vector<Eigen::VectorXd>& camera_poses);


#endif  /* _BUNDLE_ADJUSTMENT_H */