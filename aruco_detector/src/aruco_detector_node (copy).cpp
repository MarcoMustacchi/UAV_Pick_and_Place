#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

class ArucoDetector {
public:
    ArucoDetector() {
        // Initialize the ROS node handle
        nh_ = ros::NodeHandle();

        // Subscribe to the image topic
        image_sub_ = nh_.subscribe("/iris_downward_depth_camera/camera/rgb/image_raw", 1, &ArucoDetector::imageCallback, this);
        
        // Set a flag to indicate whether a marker has been detected
        marker_detected_ = false;
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        if (marker_detected_) return;

        cv_bridge::CvImagePtr cv_ptr;
        try {
            // Convert ROS image message to OpenCV Mat
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat image = cv_ptr->image;

        // Process the image to detect ArUco markers
        detectArucoMarkers(image);
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

            // Assuming that each ArUco marker corresponds to a known position in the world frame
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.05, camera_matrix_, dist_coeffs_, rvecs, tvecs);

            // Publish the detected marker's position in the world frame
            geometry_msgs::Point position;
            position.x = tvecs[0][0];
            position.y = tvecs[0][1];
            position.z = tvecs[0][2];
            ROS_INFO("Detected position is: [%f, %f, %f]",
                        position.x, position.y, position.z);

            // Check if the marker is centered in the image (i.e., the drone is directly above the marker)
            cv::Point2f marker_center = markerCorners[0][0]; // Top-left corner for simplicity
            cv::Point2f image_center(image.cols / 2.0f, image.rows / 2.0f);
            double distance_to_center = cv::norm(marker_center - image_center);

            if (distance_to_center < 50.0) { // Threshold to consider the marker centered
                // Stop the drone movement and prepare for landing
                
                // Set flag to stop further processing
                marker_detected_ = true;

                // Display the image with detected markers
                cv::imshow("Aruco Detection", image);
                cv::waitKey(0); // Wait for a key press to close the window
                cv::destroyWindow("Aruco Detection");

                // Command to stop the drone or initiate landing sequence
                ROS_INFO("Marker centered. Ready to land.");

                // Unsubscribe from the image topic
                image_sub_.shutdown();

                // Shutdown the ROS node
                ros::shutdown();

            } else {
                ROS_INFO("Marker detected but not centered. Adjusting position...");
            }
        } else {
            ROS_INFO("Aruco marker ID not detected");
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber image_sub_;
    bool marker_detected_;

    cv::Mat camera_matrix_ = (cv::Mat_<double>(3, 3) << 600, 0, 320, 0, 600, 240, 0, 0, 1);
    cv::Mat dist_coeffs_ = (cv::Mat_<double>(1, 5) << 0.1, 0.01, 0, 0, 0);  // Example distortion coefficients
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "aruco_detector");

    ArucoDetector aruco_detector;

    ros::spin();

    return 0;
}
