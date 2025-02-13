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
    // std::string left_images_dir = "/home/vio_dev/vio_workspace/src/data/";
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

    // Create instances of odometry publisher. Use a raw pointer so the topics stay active indefinitely
    OdometryPublisher* ground_truth_pub = new OdometryPublisher(this, "ground_truth_publisher");
    OdometryPublisher* visual_odometry_pub = new OdometryPublisher(this, "visual_odometry_publisher");
    // std::shared_ptr<OdometryPublisher> ground_truth_pub = std::make_shared<OdometryPublisher>(this, "ground_truth_publisher");
    // std::shared_ptr<OdometryPublisher> visual_odometry_pub = std::make_shared<OdometryPublisher>(this, "visual_odometry_publisher");
    PointCloudPublisher* pointcloud_pub = new PointCloudPublisher(this, "point_cloud_publisher");

    int number_of_images = 20;
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
    // Main visual odometry iteration
    while (rclcpp::ok() && i < image_iter_size){
        std::vector<uchar> inlier_mask;  // Initialize inlier_mask
        std::vector<uchar> status;
 
        // Load current image
        cv::Mat curr_image = cv::imread(left_images[i], cv::IMREAD_GRAYSCALE);

        std::vector<cv::Point2f> prev_points, curr_points;  // Vectors to store the coordinates of the feature points
        
        // Feature detection and tracking 
        // Detect features only when active features are below a certain threshold
        if (prev_points.size() < minimum_feature_count){ VisualOdometry::detect_features(prev_image, prev_points); }
        
        std::vector<float> err;					
        cv::Size winSize=cv::Size(21,21);																								
        cv::TermCriteria termcrit=cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01);
        cv::calcOpticalFlowPyrLK(prev_image, curr_image, prev_points, curr_points, status, err, winSize, 3, termcrit, 0, 0.001);

        // Getting rid of points for which the KLT tracking failed or those which have gone outside the frame
        int indexCorrection = 0;
        for(size_t i=0; i<status.size(); i++)
            {  cv::Point2f pt = curr_points.at(i- indexCorrection);
                if ((status.at(i) == 0)||(pt.x<0)||(pt.y<0))	{
                    if((pt.x<0)||(pt.y<0))	{
                        status.at(i) = 0;
                    }
                    prev_points.erase (prev_points.begin() + (i - indexCorrection));
                    curr_points.erase (curr_points.begin() + (i - indexCorrection));
                    indexCorrection++;
                }

            }

        // // Convert Eigen matrix to cv::Mat | Calibration matrix here is a projection matrix K[R|t]
        int calib_idx = (dataset_num[0] == '0') ? dataset_num[1]-'0' : std::stoi(dataset_num); // Get calib data index
        cv::Mat calib_proj_matrix = VisualOdometry::eigen_to_cv(calib_data[calib_idx]);
    
        // Get K(intrinsic) matrix from projection matrix
        cv::Mat left_camera_K = VisualOdometry::decompose_matrix(calib_proj_matrix);
        // cv::Mat left_camera_K = (cv::Mat_<double>(3,3) << 718.856, 0.0, 607.1928, 0.0, 718.856, 185.2157, 0.0,0.0,1.0);
        // // cv::Mat left_camera_K = (cv::Mat_<double>(3,3) << 2759.48, 0.0, 1520.69, 0.0, 2764.16, 1006.81, 0.0,0.0,1.0);
        // cv::Mat pj_matrix = (cv::Mat_<double>(3,4) << 1.0, 0.0, 0.0, 0.0, 
        //                                             0.0, 1.0, 0.0, 0.0, 
        //                                             0.0, 0.0 ,1.0, 0.0);
        // std::cout << pj_matrix << "\n";

        // Get essential matrix and inlier_mask
        essentialMatrix = cv::findEssentialMat(curr_points, prev_points, left_camera_K, cv::RANSAC, ransac_prob, 1.0, inlier_mask);
        // Get rotation and translation
        cv::recoverPose(essentialMatrix, curr_points, prev_points, left_camera_K, Rotation, Trans, inlier_mask);
        // Get scale information from ground truth
        double scale = VisualOdometry::get_scale(ground_truth, i);

        //----------------------------------------------------------------
        // Allow only forward motion and reject any sideways motion. Update pose information
        if ((scale > 0.1) && (Trans.at<double>(2) > Trans.at<double>(0)) && (Trans.at<double>(2) > Trans.at<double>(1))){
            prev_Trans = prev_Trans + /*scale*/(prev_Rotation*Trans);
            prev_Rotation = Rotation*prev_Rotation;

            // Create 3 x 4 matrix from rotation and translation
            curr_R_and_T = VisualOdometry::create_R_and_T_matrix(prev_Rotation, prev_Trans);

            // Get projection matrix by Intrisics x [R|t]
            cv::Mat prev_projection_matrix = left_camera_K * prev_R_and_T;
            cv::Mat curr_projection_matrix = left_camera_K * curr_R_and_T;

            // Triangulate points 2D points to 3D. cv.triangulatePoints gives 4D coordinates. X Y Z W. 
            // Divide XYZ by W to get 3d coordinates 
            cv::Mat points_4d;
            cv::triangulatePoints(prev_projection_matrix, curr_projection_matrix, curr_points, prev_points, points_4d);
            Eigen::MatrixXd points_3d = VisualOdometry::points_4d_to_3d(points_4d);

            // std::cout << points_4d.at<double>(0,0) << "  " << points_4d.at<double>(1,0) << "  " <<points_4d.at<double>(2,0) << "  " <<points_4d.at<double>(3,0) << "  " <<"\n";
            // std::cout << points_4d.at<double>(0,0)/points_4d.at<double>(3,0) << "  " << points_4d.at<double>(1,0)/points_4d.at<double>(3,0) << "  " <<points_4d.at<double>(2,0)/points_4d.at<double>(3,0) << "  " <<points_4d.at<double>(3,0)/points_4d.at<double>(3,0) << "  " <<"\n";

            
            // std::cout << points_3d(0,0) << "  " << points_3d(0,1) << "  " <<points_3d(0,2) <<"\n\n";

            // std::cout << points_4d << "\n\n";


            //Convert Rotation matrix to axis angle. Axis angle is parameterized to 3 values here-> (x*theta,y*theta,z*theta)
            Eigen::Vector3d axis_angle = VisualOdometry::rotation_to_axis_angle(prev_Rotation);
            double fx = left_camera_K.at<double>(0,0), fy = left_camera_K.at<double>(1,1);
            double focal_length = std::sqrt(fx*fx + fy*fy);
            // Create camera pose vector. axis angle, translation, focal length
            Eigen::VectorXd c_poses(7);
            c_poses << axis_angle[0], axis_angle[1], axis_angle[2], prev_Trans.at<double>(0),
                    prev_Trans.at<double>(1), prev_Trans.at<double>(2), focal_length;

            // Append data for Bundle adjustment
            // camera_poses.emplace_back(c_poses);  // Save camera pose
            // observations_2d.emplace_back(curr_points);  // Save 2d features
            observations_3d.emplace_back(points_3d);  // Save 3d triangulated points


            pointcloud_pub->call_publisher(observations_3d);
        //     // std::vector<std::vector<cv::Point2f>> observations_2d;
        // //     std::vector<Eigen::VectorXd> camera_poses;
            std::vector<Eigen::Vector3d> observations_3d;


        }
        //----------------------------------------------------------------


        
        // //----------------------------------------------------------------
        // if (i % 10 == 0){  // For every x images, run the bundle adjustment
        //     std::cout << observations_2d.size() << "\n";

        //     // for (int n=0; n<observations_2d.size(); n++)

        //     //     std::cout << camera_poses[n]<< "\n<----space----->\n";
        //     //     std::cout << copy_poses[n]<< "\n\n";
        //     // }


        //     std::vector<Eigen::VectorXd> copy_poses = camera_poses;
        //     auto st = cv::getTickCount();
        //     RCLCPP_INFO(this->get_logger(), "Starting Bundle Adjustment!");
        //     run_bundle_adjustment(observations_2d, observations_3d, camera_poses);
        //     auto tt = (cv::getTickCount() - st)/cv::getTickFrequency(); // How much time to run BA
        //     RCLCPP_INFO(this->get_logger(), ("Time_taken to run bundle adjustment(seconds): " + std::to_string(tt)).c_str());
        //     pointcloud_pub->call_publisher(observations_3d);

        //     // for (int n=0; n<observations_2d.size(); n++){
        //     //     std::cout << camera_poses[n]<< "\n<----space----->\n";
        //     //     std::cout << copy_poses[n]<< "\n\n";
        //     // }
            
        //     // Reset variables
        //     std::vector<std::vector<cv::Point2f>> observations_2d;
        //     std::vector<Eigen::VectorXd> camera_poses;
        //     std::vector<Eigen::Vector3d> observations_3d;
        // }
        //----------------------------------------------------------------
        // std::cout << "2d points: " << curr_points.size() << "\n";
        // std::cout << "3d points: " << points_3d.size() << "\n\n";

        // Call publisher node to publish points
        cv::Mat gt_matrix = VisualOdometry::eigen_to_cv(ground_truth[i]);
        ground_truth_pub->call_publisher(gt_matrix, "ground_truth");
        visual_odometry_pub->call_publisher(curr_R_and_T);
        RCLCPP_INFO(this->get_logger(), std::to_string(i).c_str());
        
        // std::cout << points_3d << "\n";
        // std::cout << " ----------------\n";
        // std::cout << curr_R_and_T << "outside\n";
        // std::cout << " ----------------\n";
        // Update previous image
        prev_image = curr_image;
        prev_R_and_T = curr_R_and_T;
        
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
This method converts the 4d triangulated points into 3d
input: points in 4d -> row(x,y,z,w) * column(all points)
output: points in 3d
*/
Eigen::MatrixXd VisualOdometry::points_4d_to_3d(cv::Mat& points_4d){
    // The points_4d array is flipped. It is row(x,y,z,w) * column(all points)
    // cv::Mat last_col; 
    // cv::transpose(points_4d.row(3), last_col);  // Get last row which W
    // cv::Mat points_3d(points_4d.cols, 3, CV_32F); // Create matrix with 3 columns
    // for (int i=0; i < 3; i++){
    //     cv::transpose(points_4d.row(i), points_3d.col(i));
    //     points_3d.col(i) = points_3d.col(i).mul((1/last_col)); // Divide by last column
    // }
    // Convert datatype to Eigen matrixXd
    Eigen::MatrixXd p3d;
    p3d = Eigen::MatrixXd(points_4d.cols, 3);
    // cv::cv2eigen(points_3d, p3d);


    for (int i=0; i<points_4d.cols; i++){
       double x = points_4d.at<double>(0,i);
       double y = points_4d.at<double>(1,i);
       double z = points_4d.at<double>(2,i);
       double w = points_4d.at<double>(3,i);

       p3d(i,0) = x/w;
       p3d(i,1) = y/w;
       p3d(i,2) = z/w;
    }

    return p3d;
}


/*
This method is used to convert a rotation to an axis angle
input: 3x3 Rotation matrix
output: 1x3 eigen vector
*/
Eigen::Vector3d VisualOdometry::rotation_to_axis_angle(const cv::Mat& R){
    cv::Mat W = (R - R.t()) * 0.5; 

    // Extract rotation axis
    double wx = W.at<double>(2, 1);
    double wy = W.at<double>(0, 2);   
    double wz = W.at<double>(1, 0);
    Eigen::Vector3d axis(wx, wy, wz);

    // Calculate rotation angle
    double angle = std::acos((cv::sum(R.diag())[0] - 1.0) / 2.0);

    return angle * axis; // Return axis-angle representation
}