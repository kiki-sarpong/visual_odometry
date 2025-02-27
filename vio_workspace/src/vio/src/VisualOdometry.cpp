#include <eigen3/Eigen/Core>
#include <opencv2/features2d.hpp>
#include <opencv2/core/eigen.hpp>
#include <chrono>
#include "LoadData.h"
#include "OdometryPublisher.h"
#include "VisualOdometry.h"
#include "BundleAdjustment.h"
#include "PointCloudPublisher.h"

// Constructor
VisualOdometry::VisualOdometry() : Node("visual_odometry"){
    // Run visual odometry
    run_visual_odometry();
};

// Destructor
VisualOdometry::~VisualOdometry(){};


/*
This method runs the main monocular visual odometry pipeline
input: void
returns: void
*/
void VisualOdometry::run_visual_odometry(){
    std::string dataset_num = "01";
    std::string dataset_file_path = "/home/vio_dev/vio_workspace/src/dataset/sequences/" + dataset_num + "/";
    std::string calibration_file = dataset_file_path + "calib.txt";
    std::string timestamp_file = dataset_file_path + "times.txt";
    // Set path for directories with image data
    std::string left_images_dir = "/home/vio_dev/vio_workspace/src/data/";
    // std::string left_images_dir = dataset_file_path + "image_0/";
    std::string right_images_dir = dataset_file_path + "image_1/";
    // Ground truth dataset ranges from 00 - 10.txt
    std::string ground_truth_path = "/home/vio_dev/vio_workspace/src/poses_dataset/poses/" + dataset_num + ".txt";

    LoadData data;
    std::vector<Eigen::MatrixXd> calib_data = data.load_matrix(calibration_file, "calib");
    RCLCPP_INFO(this->get_logger(), "Loaded calibration matrix data ....");
    std::vector<Eigen::MatrixXd> ground_truth = data.load_matrix(ground_truth_path);
    RCLCPP_INFO(this->get_logger(), "Loaded ground truth data ....");
    std::vector<double> timestamps = data.load_timestamps(timestamp_file);
    RCLCPP_INFO(this->get_logger(), "Loaded timestamp data ....");

    // Create instances of odometry publisher. Use a raw pointer so the topics stay active indefinitely
    OdometryPublisher* ground_truth_pub = new OdometryPublisher(this, "ground_truth_publisher");
    OdometryPublisher* visual_odometry_pub = new OdometryPublisher(this, "visual_odometry_publisher");
    // std::shared_ptr<OdometryPublisher> ground_truth_pub = std::make_shared<OdometryPublisher>(this, "ground_truth_publisher");
    // std::shared_ptr<OdometryPublisher> visual_odometry_pub = std::make_shared<OdometryPublisher>(this, "visual_odometry_publisher");
    PointCloudPublisher* pointcloud_pub = new PointCloudPublisher(this, "point_cloud_publisher");

    int number_of_images = 11;
    int& image_iter_size = number_of_images;
    std::vector<std::string> left_images = data.load_images(left_images_dir, number_of_images);

    if (number_of_images != -1){ 
        image_iter_size = number_of_images;
        RCLCPP_INFO(this->get_logger(), ("loading " + std::to_string(number_of_images)  + " images for visual odometry").c_str());
    }
    else{
        image_iter_size = timestamps.size();
        RCLCPP_INFO(this->get_logger(), ("loading " + std::to_string(timestamps.size()) + " images for visual odometry").c_str());
    }

    // Get initial image
    cv::Mat prev_image = cv::imread(left_images[0], cv::IMREAD_GRAYSCALE);

    // Initialize rotation and translation
    cv::Mat prev_Rotation = cv::Mat::eye(3, 3, CV_64F); // Identity matrix
    cv::Mat prev_Trans = cv::Mat::zeros(3, 1, CV_64F); // Start point is zero
    prev_R_and_T = VisualOdometry::create_R_and_T_matrix(prev_Rotation, prev_Trans);
    curr_R_and_T = prev_R_and_T;

    auto prev_time = cv::getTickCount();  // Get initial time count
    int i = 1;

    // Initialize SIFT with N number of features
    cv::Ptr<cv::SIFT> sift = cv::SIFT::create(5000);

    // Main visual odometry iteration
    while (rclcpp::ok() && i < image_iter_size){
        std::vector<uchar> inlier_mask;  // Initialize inlier_mask
        std::vector<uchar> status;
 
        // Load current image
        cv::Mat curr_image = cv::imread(left_images[i], cv::IMREAD_GRAYSCALE);

        std::vector<cv::Point2f> prev_points, curr_points;  // Vectors to store the coordinates of the feature points
        
        
        
        ///zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz
        // Create descriptors
        cv::Mat prev_descriptors, curr_descriptors;

        // Create keypoints
        std::vector<cv::KeyPoint> prev_keypoints, curr_keypoints;

        sift->detectAndCompute(prev_image, cv::noArray(), prev_keypoints, prev_descriptors);
        sift->detectAndCompute(curr_image, cv::noArray(), curr_keypoints, curr_descriptors);

        RCLCPP_DEBUG(this->get_logger(), "Finished sift detection.");

        // In order to use FlannBasedMatcher you need to convert your descriptors to CV_32F:
        if(prev_descriptors.type() != CV_32F) {
            prev_descriptors.convertTo(prev_descriptors, CV_32F);
        }

        if(curr_descriptors.type() != CV_32F) {
            curr_descriptors.convertTo(curr_descriptors, CV_32F);
        }


        std::vector<std::vector<cv::DMatch>> matches;  // Get matches
        // Initialize flann parameters
        cv::Ptr<cv::flann::IndexParams> index_params = cv::makePtr<cv::flann::KDTreeIndexParams>(5);
        cv::Ptr<cv::flann::SearchParams> search_prams = cv::makePtr<cv::flann::SearchParams>(100);

        cv::FlannBasedMatcher flannMatcher(index_params, search_prams); // Use the flann based matcher
        flannMatcher.knnMatch(prev_descriptors, curr_descriptors, matches, 2);

        RCLCPP_DEBUG(this->get_logger(), "Finished flanndetection detection.");

        std::vector<cv::DMatch> good_matches;  // Get good matches
        for(size_t i = 0; i < matches.size(); i++){
            const cv::DMatch& m = matches[i][0];
            const cv::DMatch& n = matches[i][1];
            if (m.distance < 0.7 * n.distance){  // Relaxed Lowe's ratio test for more matches
                good_matches.push_back(m);
            }
        }

        std::cout << "good matches after ratio test  " << good_matches.size() << "\n\n";

        // // Create prev_q and curr_q using the good matches | The good keypoints within the threshold
        for (const cv::DMatch& m : good_matches) {
            prev_points.emplace_back(prev_keypoints[m.queryIdx].pt);  // Get points from the first image
            curr_points.emplace_back(curr_keypoints[m.trainIdx].pt);  // Get points from the second image
        }
        //zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz

        
    



        // // ---------------------------------------------------------------
        // // Feature detection and tracking 
        // // Detect features only when active features are below a certain threshold
        // if (prev_points.size() < minimum_feature_count){ VisualOdometry::detect_features(prev_image, prev_points); }
        
        // std::vector<float> err;					
        // cv::Size winSize=cv::Size(21,21);																								
        // cv::TermCriteria termcrit=cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01);
        // cv::calcOpticalFlowPyrLK(prev_image, curr_image, prev_points, curr_points, status, err, winSize, 3, termcrit, 0, 0.001);

        // // Getting rid of points for which the KLT tracking failed or those which have gone outside the frame
        // int indexCorrection = 0;
        // for(size_t i=0; i<status.size(); i++)
        //     {  cv::Point2f pt = curr_points.at(i- indexCorrection);
        //         if ((status.at(i) == 0)||(pt.x<0)||(pt.y<0))	{
        //             if((pt.x<0)||(pt.y<0))	{
        //                 status.at(i) = 0;
        //             }
        //             prev_points.erase (prev_points.begin() + (i - indexCorrection));
        //             curr_points.erase (curr_points.begin() + (i - indexCorrection));
        //             indexCorrection++;
        //         }

        //     }
        // // ---------------------------------------------------------------

        // // Convert Eigen matrix to cv::Mat | Calibration matrix here is a projection matrix K[R|t]
        int calib_idx = (dataset_num[0] == '0') ? dataset_num[1]-'0' : std::stoi(dataset_num); // Get calib data index
        cv::Mat calib_proj_matrix = VisualOdometry::eigen_to_cv(calib_data[calib_idx]);
    
        // Get K(intrinsic) matrix from projection matrix
        // cv::Mat left_camera_K = VisualOdometry::decompose_matrix(calib_proj_matrix);
        cv::Mat left_camera_K = (cv::Mat_<double>(3,3) << 2759.48, 0.0, 1520.69, 0.0, 2764.16, 1006.81, 0.0,0.0,1.0);

        // cv::Mat pj_matrix = (cv::Mat_<double>(3,4) << 1.0, 0.0, 0.0, 0.0, 
        //                                             0.0, 1.0, 0.0, 0.0, 
        //                                             0.0, 0.0 ,1.0, 0.0);
        // std::cout << pj_matrix << "\n";

        // Get essential matrix and inlier_mask
        // essentialMatrix = cv::findEssentialMat(curr_points, prev_points, left_camera_K, cv::RANSAC, ransac_prob, 1.0, inlier_mask);
        essentialMatrix = cv::findEssentialMat(prev_points, curr_points, left_camera_K, cv::RANSAC, ransac_prob, 1.0, inlier_mask);
        // Get rotation and translation
        // cv::recoverPose(essentialMatrix, curr_points, prev_points, left_camera_K, Rotation, Trans, inlier_mask);
        cv::recoverPose(essentialMatrix, prev_points, curr_points, left_camera_K, Rotation, Trans, inlier_mask);

        // Get scale information from ground truth
        double scale = VisualOdometry::get_scale(ground_truth, i);


        //----------------------------------------------------------------
        // Allow only forward motion and reject any sideways motion. Update pose information
        // if ((scale > 0.1) && (Trans.at<double>(2) > Trans.at<double>(0)) && (Trans.at<double>(2) > Trans.at<double>(1))){

            prev_Trans = prev_Trans + /*scale*/(prev_Rotation*Trans);
            prev_Rotation = Rotation*prev_Rotation;

            // Create 3 x 4 matrix from rotation and translation
            curr_R_and_T = VisualOdometry::create_R_and_T_matrix(prev_Rotation, prev_Trans);

            // cv::Mat pj_matrix = (cv::Mat_<double>(3,4) << 2954.20319, -81.0943874, 1092.39880, 2652.99235, 
            //                                         211.471952, 2746.15539, 1033.51685, 46.5317670, 
            //                                         0.148675289, -0.0169368278 ,0.988741019, -0.0647342517);
            
            // Get projection matrix by Intrisics x [R|t]
            cv::Mat prev_projection_matrix = left_camera_K * prev_R_and_T;
            cv::Mat curr_projection_matrix = left_camera_K * curr_R_and_T;
            // cv::Mat curr_projection_matrix = pj_matrix;
            // std::cout << "--------------------" << "\n";
  
            // std::cout << Rotation << "\n\n";
            // std::cout << Trans << "\n\n";
            // std::cout << prev_projection_matrix << "\n\n";
            // std::cout << curr_projection_matrix << "\n\n";
            // std::cout << "--------------------" << "\n";
  
            // Triangulate points 2D points to 3D. cv.triangulatePoints gives 4D coordinates. X Y Z W. 
            // Divide XYZ by W to get 3d coordinates 
            cv::Mat points_4d;

            // std::vector<cv::Point2f> test_prev, test_curr;
            // std::vector<cv::Point2f> test_prev(prev_points.begin(), prev_points.begin() + 200);
            // std::vector<cv::Point2f> test_curr(curr_points.begin(), curr_points.begin() + 200);

            // for (int a = 0; a < 50; a++){
            //     test_prev.push_back(prev_points[a]);
            //     test_curr.push_back(test_curr[a]);
            //     std::cout << "working \n";
            // }

            // test_curr.push_back(cv::Point2f(2782.4656,1676.994));
            // test_curr.push_back(cv::Point2f(170.51485,336.6516));
            // test_curr.push_back(cv::Point2f(20.600048,306.6135));
            // test_curr.push_back(cv::Point2f(130.22656,1812.8231));
            // test_curr.push_back(cv::Point2f(189.02013,37.78019));

            // test_prev.push_back(cv::Point2f(36.814003, 862.4743));
            // test_prev.push_back(cv::Point2f(218.9255, 339.8642));
            // test_prev.push_back(cv::Point2f(224.93124, 345.42947));
            // test_prev.push_back(cv::Point2f(229.50792, 1709.9233));
            // test_prev.push_back(cv::Point2f(231.55942,55.260323));

            //******************************************** */
            // prev_points = test_prev;
            // curr_points = test_curr;
            // curr_projection_matrix = pj_matrix;

            cv::triangulatePoints(prev_projection_matrix, curr_projection_matrix, prev_points, curr_points, points_4d);
            // cv::triangulatePoints(prev_projection_matrix, curr_projection_matrix, prev_points, curr_points, points_4d);

            // std::cout << points_4d.at<float>(0,0) << "  " << points_4d.at<float>(1,0) << "  " <<points_4d.at<float>(2,0) << "  " <<points_4d.at<float>(3,0) << "  " <<"\n";
            
            Eigen::MatrixXd points_3d = VisualOdometry::points_4d_to_3d(points_4d);

            // std::cout << "printing points start " << "\n";
            // std::cout << points_3d(0,0) << "  " << points_3d(0,1) << "  " <<points_3d(0,2) << "  " <<"\n";
            // std::cout << "printing points end " << "\n";
            // std::cout << "############################################################" << "\n\n\n";
            // std::cout << points_3d(0,0) << "  " << points_3d(0,1) << "  " <<points_3d(0,2) <<"\n\n";

            // std::vector<cv::Point2f> observations_2d;
            // std::vector<Eigen::VectorXd> camera_poses;
            // std::vector<int> camera_indices;
            // Eigen::MatrixXd observations_3d = points_3d;

            // Concatenate 3d matrix
            if (i == 1){
                observations_3d = points_3d;
            }
            else{
                Eigen::MatrixXd hold_3d = observations_3d; // Temporarily hold the data
                observations_3d.resize((hold_3d.rows() + points_3d.rows()), points_3d.cols());
                observations_3d << hold_3d,
                                points_3d;

            // std::cout << "1 " << hold_3d.rows() <<"\n";
            // std::cout << "2 " << hold_3d.cols() <<"\n";
            }
            
            
            // std::cout << "3 " << observations_3d.rows() <<"\n";
            // std::cout << "3.5 " << observations_3d.cols() <<"\n";
            // std::cout << "4 " << points_3d.rows() <<"\n";

            // Do vertical concatenation for points
            observations_2d.insert(observations_2d.end(), prev_points.begin(), prev_points.end());
            observations_2d.insert(observations_2d.end(), curr_points.begin(), curr_points.end());

            // Save the indices for the 2d points
            std::vector<int> p_prev(prev_points.size(), i-1); 
            std::vector<int> p_curr(curr_points.size(), i);

            // Append camera 2d observations
            camera_indices.insert(camera_indices.end(), p_prev.begin(), p_prev.end()); // Previous
            camera_indices.insert(camera_indices.end(), p_curr.begin(), p_curr.end()); // Current

            // Convert the projection matrix and focal length to a 7 parameter camera vector
            camera_poses.push_back(VisualOdometry::rotation_to_axis_angle(prev_R_and_T, left_camera_K));
            camera_poses.push_back(VisualOdometry::rotation_to_axis_angle(curr_R_and_T, left_camera_K));

            std::cout << "number of 2d_observations " << camera_indices.size()/2 <<"\n";
            std::cout << "number of camera_indices " << observations_2d.size()/2 <<"\n";
            std::cout << "number of 3d_points " << observations_3d.size()/3 <<"\n";
            std::cout << "number of camera_poses " << camera_poses.size() <<"\n";
            ///----------////
            // std::cout << points_3d << "\n";
            // std::cout << "##################" << "\n";
            // for (int a =0; a < 100; a++){
            //     std::cout << curr_points[a] << "\n";
            // }
            // for (int a=0; a < observations_2d.size(); a++){
            //     std::cout << observations_2d[a].x << "   " << observations_2d[a].y << "   " << camera_indices[a] <<"\n";
            // }
            // std::cout << "##################" << "\n";

            // return ;
        //----------------------------------------------------------------


        
        //----------------------------------------------------------------
        if (i % 5 == 0 ){
            auto st = cv::getTickCount();
            RCLCPP_INFO(this->get_logger(), "Starting Bundle Adjustment!");
            // Run bundle adjustment
            // run_bundle_adjustment(observations_2d, observations_3d, camera_poses, camera_indices);
            auto tt = (cv::getTickCount() - st)/cv::getTickFrequency(); // How much time to run BA
            RCLCPP_INFO(this->get_logger(), ("Time_taken to run bundle adjustment(seconds): " + std::to_string(tt)).c_str());  
        }
        // ----------------------------------------------------------------


        // return;


        // Call publisher node to publish points
        cv::Mat gt_matrix = VisualOdometry::eigen_to_cv(ground_truth[i]);
        ground_truth_pub->call_publisher(gt_matrix, "ground_truth");
        visual_odometry_pub->call_publisher(curr_R_and_T);
        pointcloud_pub->call_publisher(observations_3d);
        RCLCPP_INFO(this->get_logger(), std::to_string(i).c_str());
        
        // std::cout << points_3d << "\n";
        // std::cout << " ----------------\n";
        // std::cout << curr_R_and_T << "outside\n";
        // std::cout << " ----------------\n";
        // Update previous image
        prev_image = curr_image;
        prev_R_and_T = curr_R_and_T;
        // prev_Rotation = curr_Rotation;
        // prev_Trans = curr_Trans;
        
        // Calculate frames per sec   
        auto curr_time = cv::getTickCount();
        auto totaltime = (curr_time - prev_time) / cv::getTickFrequency();
        auto FPS = 1.0 / totaltime;
        prev_time = curr_time;
        i++;    // Increment count
        RCLCPP_DEBUG(this->get_logger(), ("Frames per sec: " + std::to_string(int(FPS))).c_str());
    }
    RCLCPP_INFO(this->get_logger(), "Visual odometry complete!");

}

/*
This method decomposes a projection matrix to extract the camera intrinsics
input: projection matrix
returns: camera intrinsics
*/
cv::Mat VisualOdometry::decompose_matrix(cv::Mat& projection_matrix){
    // Matrices to store the outputs
    cv::Mat K, rotMatrix, transVect;
    cv::Mat rotMatrixX, rotMatrixY, rotMatrixZ, t;

    // Decompose the projection matrix
    cv::decomposeProjectionMatrix(projection_matrix, K, rotMatrix, transVect,
                                  rotMatrixX, rotMatrixY, rotMatrixZ, t);

    return K;
}


/*
This method converts an eigen matrix to a cv matrix
input: eigen matrix
returns: cv matrix
*/
cv::Mat VisualOdometry::eigen_to_cv(Eigen::MatrixXd& eigen_matrix){
    // Create projection matrix
    cv::Mat cv_matrix;
    cv::eigen2cv(eigen_matrix, cv_matrix);
    return cv_matrix;
}


/*
This method creates a 3 by 4 matrix from a rotation and translation
inputs: rotation, translation
returns: 3 x 4 matrix
*/
cv::Mat VisualOdometry::create_R_and_T_matrix(cv::Mat& rotation, cv::Mat& translation){
    cv::Mat matrix_ = cv::Mat::zeros(3, 4, CV_64F); // Create 4 by 4 matrix
    // Create 3 x 4 matrix
    rotation.copyTo(matrix_(cv::Rect(0, 0, 3, 3)));
    translation.copyTo(matrix_(cv::Rect(3, 0, 1, 3)));

    return matrix_;
}


/*
This method gets the absolute scale information from the ground truth data and uses 
    that to fit the monocular visual odometry estimations.
You can substitute the ground truth data with any sensor that gives absolute scale/depth information
input: vector of ground truth eigen matrices
output: scale information
*/
double VisualOdometry::get_scale(std::vector<Eigen::MatrixXd>& eigen_matrix_vector, const int idx){
    // Get current and previous ground truth translation
    Eigen::VectorXd curr_gt = eigen_matrix_vector[idx].col(eigen_matrix_vector[idx].cols() - 1);
    Eigen::VectorXd prev_gt = eigen_matrix_vector[idx-1].col(eigen_matrix_vector[idx-1].cols() - 1);

    // Calculate the difference and find the norm
    Eigen::VectorXd diff = curr_gt - prev_gt;
    return diff.norm();
}


/*
This method detects features with the FAST algorithm
input: image, points
output: void
*/
void VisualOdometry::detect_features(cv::Mat& image, std::vector<cv::Point2f>& points){
    std::vector<cv::KeyPoint> keypoints;   // Create keypoints
    cv::FAST(image, keypoints, fast_threshold, nonmaxSuppression);
    cv::KeyPoint::convert(keypoints, points, std::vector<int>());
}


/*
This method converts the 4d triangulated points into 3d points
input: points in 4d -> row(x,y,z,w) * column(all points)
output: points in 3d
*/
Eigen::MatrixXd VisualOdometry::points_4d_to_3d(cv::Mat& points_4d){
    // The points_4d array is flipped. It is row(x,y,z,w) * column(all points)
    // Convert datatype to Eigen matrixXd
    Eigen::MatrixXd p3d;
    p3d = Eigen::MatrixXd(points_4d.cols, 3);
    // cv::cv2eigen(points_3d, p3d);

    for (int i=0; i<points_4d.cols; i++){
        // Use <float> instead of <double>. cv::point2f.. <double> gives wrong values
       double x = points_4d.at<float>(0,i);
       double y = points_4d.at<float>(1,i);
       double z = points_4d.at<float>(2,i);
       double w = points_4d.at<float>(3,i);
       p3d(i,0) = x/w;
       p3d(i,1) = y/w;
       p3d(i,2) = z/w;

    }
    return p3d;
}


/*
This method is used to convert a rotation, translation and focal length into a camera vector
The camera vector is the camera pose inputs for bundle adjustment
input: 3x4 matrix (rotation and translation), intrinsics matrix
output: 1d eigen vector
*/
Eigen::VectorXd VisualOdometry::rotation_to_axis_angle(const cv::Mat& matrix_RT, const cv::Mat& K){
    // Get Rotation and Translation from matrix_RT
    cv::Mat Rotation = matrix_RT(cv::Range(0, 3), cv::Range(0, 3));
    cv::Mat Translation = matrix_RT(cv::Range(0, 3), cv::Range(3, 4));

    Eigen::MatrixXd eigen_rotation;
    cv::cv2eigen(Rotation, eigen_rotation);
    double axis_angle[3];
    // Convert rotation matrix to axis angle
    ceres::RotationMatrixToAngleAxis<double>(eigen_rotation.data(), axis_angle);

    // Find focal length
    double fx = K.at<double>(0,0), fy = K.at<double>(1,1);
    double focal_length = std::sqrt(fx*fx + fy*fy);

    // Create camera pose vector = axis angle, translation, focal length
    Eigen::VectorXd camera_vector(7);
    camera_vector << axis_angle[0], axis_angle[1], axis_angle[2], Translation.at<double>(0),
            Translation.at<double>(1), Translation.at<double>(2), focal_length;

    return camera_vector;
}