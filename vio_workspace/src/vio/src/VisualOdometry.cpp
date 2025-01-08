#include <eigen3/Eigen/Core>
#include <opencv2/features2d.hpp>
#include <opencv2/core/eigen.hpp>
#include "LoadData.h"
#include "OdometryPublisher.h"
#include "VisualOdometry.h"

// Constructor
VisualOdometry::VisualOdometry() : Node("visual_odometry"){
    // Run visual odometry
    run_visual_odometry();
};

// Destructor
VisualOdometry::~VisualOdometry(){};


/*
This method runs the main visual odometry pipeline
input: void
returns: void
*/
void VisualOdometry::run_visual_odometry(){
    std::string dataset_num = "00";
    std::string dataset_file_path = "/home/vio_dev/vio_workspace/src/dataset/sequences/" + dataset_num + "/";
    std::string calibration_file = dataset_file_path + "calib.txt";
    std::string timestamp_file = dataset_file_path + "times.txt";
    // Set path for directories with image data
    std::string left_images_dir = dataset_file_path + "image_0/";
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

    // Create instances of odometry publisher
    OdometryPublisher* ground_truth_pub = new OdometryPublisher(this, "ground_truth_publisher");
    OdometryPublisher* visual_odometry_pub = new OdometryPublisher(this, "visual_odometry_publisher");
    // std::shared_ptr<OdometryPublisher> ground_truth_pub = std::make_shared<OdometryPublisher>(this, "ground_truth_publisher");
    // std::shared_ptr<OdometryPublisher> visual_odometry_pub = std::make_shared<OdometryPublisher>(this, "visual_odometry_publisher");

    int number_of_images = 1000;
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

    // Initialize elements
    // Get initial image
    cv::Mat prev_image = cv::imread(left_images[0], cv::IMREAD_GRAYSCALE);

    // Create  detector
    cv::Ptr<cv::SIFT> detector = cv::SIFT::create();

    // Initialize rotation and translation
    cv::Mat prev_Rotation = cv::Mat::eye(3, 3, CV_64F); // Identity matrix
    cv::Mat prev_Trans = cv::Mat::zeros(3, 1, CV_64F); // Start point is zero
    prev_R_and_T = VisualOdometry::create_R_and_T_matrix(prev_Rotation, prev_Trans);

    int i = 1;
    // Main visual odometry iteration
    while (rclcpp::ok() && i < image_iter_size){
        std::vector<uchar> inlier_mask;  // Initialize inlier_mask
        std::vector<cv::DMatch> matches;  // Get matches
        std::vector<cv::DMatch> good_matches;  // Get good matches
        // Create prev_q and curr_q keypoints using the good matches
        std::vector<cv::Point2f> prev_q, curr_q; 
        // Create keypoints
        std::vector<cv::KeyPoint> prev_keypoints, curr_keypoints;
        // Load current image
        cv::Mat curr_image = cv::imread(left_images[i], cv::IMREAD_GRAYSCALE);

        detector->detectAndCompute(prev_image, cv::noArray(), prev_keypoints, prev_descriptors);
        detector->detectAndCompute(curr_image, cv::noArray(), curr_keypoints, curr_descriptors);

        RCLCPP_DEBUG(this->get_logger(), "Finished detection.");

        // In order to use FlannBasedMatcher you need to convert your descriptors to CV_32F:
        if(prev_descriptors.type() != CV_32F) {
            prev_descriptors.convertTo(prev_descriptors, CV_32F);
        }

        if(curr_descriptors.type() != CV_32F) {
            curr_descriptors.convertTo(curr_descriptors, CV_32F);
        }
        
        



        // // Filter matches based on threshold
        // for (size_t i = 0; i < matches.size(); ++i) {
        //     const cv::DMatch& m = matches[i];
        //     if (i + 1 < matches.size()) { 
        //         const cv::DMatch& n = matches[i + 1];  // Assuming 'n' is the next match (second best)
        //         if (m.distance < this->feature_detection_prob * n.distance) {
        //             good_matches.emplace_back(m);
        //         }
        //     }
        // }


        // Matcher
        cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
        std::vector< std::vector<cv::DMatch>> knn_matches;
        matcher->knnMatch(prev_descriptors, curr_descriptors, knn_matches, 2);
    
        // Filter matches using the Lowe's ratio test
        for (size_t i = 0; i < knn_matches.size(); i++)
        {
            if (knn_matches[i][0].distance < feature_detection_prob * knn_matches[i][1].distance)
            {
                good_matches.push_back(knn_matches[i][0]);
            }
        }



        // Create prev_q and curr_q using the good matches | The good keypoints within the threshold
        for (const cv::DMatch& m : good_matches) {
            prev_q.emplace_back(prev_keypoints[m.queryIdx].pt);  // Get points from the first image
            curr_q.emplace_back(curr_keypoints[m.trainIdx].pt);  // Get points from the second image
        }
        // Skip iteration if there are too few matches for essential matrix
        if (good_matches.size() < 5) {
            continue;  
        }

        // Convert Eigen matrix to cv::Mat | Calibration matrix here is a projection matrix K[R|t]
        cv::Mat calib_proj_matrix = VisualOdometry::eigen_to_cv(calib_data[0]); //!!!  FIX
    
        // Get K(intrinsic) matrix from projection matrix
        cv::Mat left_camera_K = VisualOdometry::decompose_matrix(calib_proj_matrix);

        // Get essential matrix and inlier_mask
        essentialMatrix = cv::findEssentialMat(prev_q, curr_q, left_camera_K, cv::RANSAC, ransac_prob, 1.0, inlier_mask);
        // Get rotation and translation
        cv::recoverPose(essentialMatrix, prev_q, curr_q, left_camera_K, Rotation, Trans, inlier_mask);

        cv::Mat inverse;
        cv::invert(Rotation, inverse, cv::DECOMP_LU);
        prev_Trans = prev_Trans - prev_Rotation*Trans;
        prev_Rotation = prev_Rotation*inverse;
        
        // prev_Trans = prev_Trans + prev_Rotation*Trans;
        // prev_Rotation = Rotation*prev_Rotation;
        
        // Create 3 x 4 matrix from rotation and translation
        curr_R_and_T = VisualOdometry::create_R_and_T_matrix(prev_Rotation, prev_Trans);

        // Update the rotation and translation based on the new output
        // VisualOdometry::update_pose(prev_R_and_T, curr_R_and_T); 

        // Get projection matrix by Intrisics x [R|t]
        // cv::Mat prev_projection_matrix = left_camera_K * prev_R_and_T;
        // cv::Mat curr_projection_matrix = left_camera_K * curr_R_and_T;

        // Triangulate points 2D points to 3D
        // cv::triangulatePoints(prev_projection_matrix, curr_projection_matrix, prev_q, curr_q, triangulated_points);

        // Call publisher node to publish points
        cv::Mat gt_matrix = VisualOdometry::eigen_to_cv(ground_truth[i]);
        ground_truth_pub->call_publisher(gt_matrix, "ground_truth");
        visual_odometry_pub->call_publisher(curr_R_and_T);
        RCLCPP_INFO(this->get_logger(), std::to_string(i).c_str());
        
        // std::cout << triangulated_points << "\n";
        // std::cout << " ----------------\n";
        // std::cout << curr_R_and_T << "outside\n";
        // std::cout << " ----------------\n";
        // Update previous image
        prev_image = curr_image;
        prev_R_and_T = curr_R_and_T;
        // prev_Rotation = Rotation;
        // prev_Trans = Trans;
        i++;
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
This method updates the pose(rotation, translation) for the visual odometry
inputs: Prev_r_and_t(3x4 matrix), curr_r_and_t(3x4 matrix)
returns: void
*/
void VisualOdometry::update_pose(cv::Mat& prev_mat, cv::Mat& curr_mat){
     // Create 4 by 4 matrix
    cv::Mat prev_h_matrix = cv::Mat::zeros(4, 4, CV_64F);
    cv::Mat curr_h_matrix = cv::Mat::zeros(4, 4, CV_64F);
    
    // std::cout << curr_mat << "first currmat\n";
    // std::cout << " ----------------\n";

    // Convert to homogeneous matrix. cv::Rect(x,y,width,height)
    prev_mat.copyTo(prev_h_matrix(cv::Rect(0, 0, 4, 3)));
    curr_mat.copyTo(curr_h_matrix(cv::Rect(0, 0, 4, 3)));
    prev_h_matrix.at<double>(3, 3) = 1.0;
    curr_h_matrix.at<double>(3, 3) = 1.0;
    // std::cout << curr_h_matrix << "second currmat\n";
    // std::cout << " ----------------\n";

    // cv::Mat inverse_mat_h;
    // cv::invert(curr_h_matrix, inverse_mat_h, cv::DECOMP_LU);
    cv::Mat new_prev_h = prev_h_matrix * curr_h_matrix;
    std::cout << new_prev_h << "third currmat\n";
    std::cout << " ----------------\n";

    prev_mat = new_prev_h(cv::Rect(0, 0, 4, 3));
}