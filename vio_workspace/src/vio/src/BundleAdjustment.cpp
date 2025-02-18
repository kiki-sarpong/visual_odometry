#include "BundleAdjustment.h"


/*
This method runs the bundle adjustment pipeline to minimize reprojection error
input: vector of 2d feature points, vector of 3d triangulated points, vector of camera poses
returns: void
*/
void run_bundle_adjustment(std::vector<std::vector<cv::Point2f>>& observations_2d, std::vector<Eigen::MatrixXd>& observations_3d, 
    std::vector<Eigen::VectorXd>& camera_poses){

    // ceres::Problem problem;
    // ceres::CostFunction* cost_function;
    // const int cam_size = 7;
    // const int points_2d_size = 2;
    // const int points_3d_size = 3;

    // Add the camera poses to the parameter block
    // for (auto& cam : camera_poses){
    //     /*   Using ".data()" because the function expects a double* pointer   */
    //     problem.AddParameterBlock(cam.data(), cam_size);
    // }



    for (int v_num=0; v_num<observations_3d.size(); v_num++){
        ceres::Problem problem;
        ceres::CostFunction* cost_function;
        const int cam_size = 7;
        const int points_2d_size = 2;
        const int points_3d_size = 3;

        // int indx = 0;
        // for(std::vector<cv::Point2f>& points : observations_2d){
 
        Eigen::Vector3d coordinates[observations_3d[v_num].size()];

        for (int indx=0; indx<observations_3d[v_num].rows(); indx++){

            coordinates[indx] = {observations_3d[v_num](indx, 0), observations_3d[v_num](indx, 1), observations_3d[v_num](indx,2)};

            // std::cout << indx << "   "  << observations_3d[v_num].cols() <<  " " <<observations_2d[v_num].size()<< "\n";
            // std::cout << coordinates[indx]  << "\n";
            // std::cout << observations_3d[v_num](indx) << "\n";
            // return;

            problem.AddParameterBlock(coordinates[indx].data(), points_3d_size); 
            problem.AddParameterBlock(camera_poses[v_num].data(), cam_size);

            for(size_t i=0; i < observations_2d[v_num].size(); i++){ /* Iterate through all the 2d points per image*/ 
                // Eigen::Vector3d v3d;
                // v3d << observations_3d[indx](i,0), observations_3d[indx](i,1), observations_3d[indx](i,2);
                // problem.AddParameterBlock(v3d.data(), 3);


                float& x = observations_2d[v_num][i].x;
                float& y = observations_2d[v_num][i].y;
                BundleAdjustment* b_adj_ptr = new BundleAdjustment(x/*x*/, y/*y*/);
                cost_function = new ceres::AutoDiffCostFunction<BundleAdjustment, points_2d_size, cam_size, points_3d_size>(b_adj_ptr);
                problem.AddResidualBlock(cost_function, nullptr/*squared_loss*/, camera_poses[v_num].data(), coordinates[indx].data());
            }

            // Increment index
            // indx++;
        }
        std::cout << "starting solution" << "\n";
        // return;
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_SCHUR; //ceres::SPARSE_NORMAL_CHOLESKY said to be slower;
        options.minimizer_progress_to_stdout = false;
        options.max_num_iterations = 100;
        options.num_threads = 4;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        std::cout << summary.BriefReport() << "\n";

        // std::cout << "starting here" << "\n";
        // Reassign values
        for (int id=0; id<observations_3d[v_num].size(); id++){
            std::cout << "coordinates[indx]"  << "\n";
            observations_3d[v_num](id, 0) = coordinates[id][0];
            observations_3d[v_num](id, 1) = coordinates[id][1];
            observations_3d[v_num](id, 2) = coordinates[id][2];
        }





    }
    // ceres::Solver::Options options;
    // options.linear_solver_type = ceres::DENSE_SCHUR; //ceres::SPARSE_NORMAL_CHOLESKY said to be slower;
    // options.minimizer_progress_to_stdout = false;
    // options.max_num_iterations = 100;
    // options.num_threads = 4;

    // ceres::Solver::Summary summary;
    // ceres::Solve(options, &problem, &summary);
    // std::cout << summary.BriefReport() << "\n";





}