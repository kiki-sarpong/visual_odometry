#include "BundleAdjustment.h"


/*
This method runs the bundle adjustment pipeline to minimize reprojection error
input: vector of 2d feature points, vector of 3d triangulated points, vector of camera poses
returns: void
*/
void run_bundle_adjustment(std::vector<cv::Point2f>& observations_2d, Eigen::MatrixXd& observations_3d, 
    std::vector<Eigen::VectorXd>& camera_poses, std::vector<int>& camera_indices){

        ceres::Problem problem;
        ceres::CostFunction* cost_function;
        const int cam_size = 7;
        const int points_2d_size = 2;
        const int points_3d_size = 3;

        // Add the camera poses to the parameter block
        // for (const int& frame_id : camera_indices){
        //     /*   Using ".data()" because the function expects a double* pointer   */
        //     problem.AddParameterBlock(camera_poses[frame_id].data(), cam_size);
        // }


        Eigen::Vector3d coordinates[observations_3d.rows()];
        bool is_camera_locked = false;

        for (int indx=0; indx<observations_3d.rows(); indx++){

            coordinates[indx] = {observations_3d(indx, 0), observations_3d(indx, 1), observations_3d(indx,2)};

            // std::cout << coordinates[indx]  << "\n";

            // problem.AddParameterBlock(coordinates[indx].data(), points_3d_size); 

            for(size_t i=0; i < observations_2d.size(); i++){ /* Iterate through all the 2d points per image*/ 
                float& x = observations_2d[i].x;
                float& y = observations_2d[i].y;
                int frame_id = camera_indices[i];

                BundleAdjustment* b_adj_ptr = new BundleAdjustment(x/*x*/, y/*y*/);
                cost_function = new ceres::AutoDiffCostFunction<BundleAdjustment, points_2d_size, cam_size, points_3d_size>(b_adj_ptr);
                problem.AddResidualBlock(cost_function, nullptr/*squared_loss*/, camera_poses[frame_id].data(), coordinates[indx].data());
            
                // Lock the first camera to better deal with scene orientation ambiguity
                if (! is_camera_locked){
                    problem.SetParameterBlockConstant(camera_poses[frame_id].data());
                    is_camera_locked = true;
                }
            }

        }
        std::cout << "starting solution" << "\n";
        // return;
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_SCHUR; //ceres::SPARSE_NORMAL_CHOLESKY said to be slower;
        options.minimizer_progress_to_stdout = true;
        options.max_num_iterations = 100;
        // options.num_threads = 4;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        std::cout << summary.BriefReport() << "\n";

        // std::cout << "starting here" << "\n";
        // Reassign values
        for (int id=0; id<observations_3d.rows(); id++){
            observations_3d(id, 0) = coordinates[id][0];
            observations_3d(id, 1) = coordinates[id][1];
            observations_3d(id, 2) = coordinates[id][2];
        }
        // std::cout << observations_3d << "\n";

}