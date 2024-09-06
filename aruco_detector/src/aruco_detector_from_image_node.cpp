#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

class ArucoDetector {
public:
    ArucoDetector() {
        // Set up camera matrix and distortion coefficients (example values)
        camera_matrix_ = (cv::Mat_<double>(3, 3) << 600, 0, 320, 0, 600, 240, 0, 0, 1);
        dist_coeffs_ = (cv::Mat_<double>(1, 5) << 0.1, 0.01, 0, 0, 0);

        // Load and process the image from the file system
        std::string image_path = "/home/musta/catkin_ws/src/aruco_detector/images/image.png";
        cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR);

        if (image.empty()) {
            ROS_ERROR("Could not open or find the image!");
            return;
        }
        
        // Display the image
        cv::namedWindow("Aruco Image", cv::WINDOW_NORMAL); // using cv::WINDOW_NORMAL, I can resize manually the image
        cv::imshow("Aruco Image", image);
        cv::waitKey(0); // Wait for a key press to close the window

        // Process the image to detect ArUco markers
        detectArucoMarkers(image);

        // Display the image with detected markers
        cv::namedWindow("Aruco Detection", cv::WINDOW_NORMAL); // using cv::WINDOW_NORMAL, I can resize manually the image
        cv::imshow("Aruco Detection", image);
        cv::waitKey(0); // Wait for a key press to close the window
    }

    void detectArucoMarkers(cv::Mat& image) {
        // Load predefined dictionary of markers
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);

        // Detect markers
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners;

        cv::aruco::detectMarkers(image, dictionary, markerCorners, markerIds);

        // If markers are detected, draw them on the image
        if (!markerIds.empty()) {
            cv::aruco::drawDetectedMarkers(image, markerCorners, markerIds);

            // For simplicity, let's assume we are interested in the first detected marker
            int marker_id = markerIds[0];

            ROS_INFO("Detected ArUco marker ID: %d", marker_id);

            // Estimate pose if needed
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.05, camera_matrix_, dist_coeffs_, rvecs, tvecs);

            // Output the detected marker's position in the camera frame
            ROS_INFO("Detected position is: [x: %f, y: %f, z: %f]",
                        tvecs[0][0], tvecs[0][1], tvecs[0][2]);

            // Optionally, draw the axis for the pose estimation
            cv::aruco::drawAxis(image, camera_matrix_, dist_coeffs_, rvecs[0], tvecs[0], 0.1);
        } else {
            ROS_INFO("No ArUco markers detected.");
        }
    }

private:
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "aruco_detector");

    ArucoDetector aruco_detector;

    return 0;
}