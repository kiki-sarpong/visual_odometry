#include "BundleAdjustment.h"


/*
This method runs the bundle adjustment pipeline to minimize reprojection error
input: vector of 2d feature points, vector of 3d triangulated points, vector of camera poses
returns: void
*/
void run_bundle_adjustment(std::vector<std::vector<cv::Point2f>>& observations_2d, std::vector<Eigen::Vector3d>& observations_3d, 
    std::vector<Eigen::VectorXd>& camera_poses){

    ceres::Problem problem;
    ceres::CostFunction* cost_function;
    const int cam_size = 7;
    const int points_2d_size = 2;
    const int points_3d_size = 3;

    // // Add the camera poses to the parameter block
    // for (auto& cam : camera_poses){
    //     /* Using ".data()" because the function expects a double* pointer*/
    //     problem.AddParameterBlock(cam.data(), cam_size);
    // }

    int indx = 0;
    for(std::vector<cv::Point2f>& points : observations_2d){
        // Continuously add the 3d points for every image through the iteration
        problem.AddParameterBlock(observations_3d[indx].data(), 3); 
        problem.AddParameterBlock(camera_poses[indx].data(), cam_size);

        for(size_t i=0; i < points.size(); i++){ /* Iterate through all the 2d points per image*/
            BundleAdjustment* b_adj_ptr = new BundleAdjustment(points[i].x/*x*/, points[i].y/*y*/);
            cost_function = new ceres::AutoDiffCostFunction<BundleAdjustment, points_2d_size, cam_size, points_3d_size>(b_adj_ptr);
            problem.AddResidualBlock(cost_function, nullptr/*squared_loss*/, camera_poses[indx].data(), observations_3d[indx].data());
        }

        // Increment index
        indx++;
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = false;
    options.max_num_iterations = 100;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";
}