#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include "aruco_detector/DetectArucoMarker.h" // Custom service message

// Global variables to store the latest image and detection results
sensor_msgs::ImageConstPtr latest_image;
bool image_received = false;
int detected_marker_id = -1;

// Callback function to store the latest image
void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    latest_image = msg;
    image_received = true;
}

// Function to detect ArUco marker in the given image
void detectArucoMarkerInImage(const sensor_msgs::ImageConstPtr& image_msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat image = cv_ptr->image;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;

    cv::aruco::detectMarkers(image, dictionary, markerCorners, markerIds);

    if (!markerIds.empty()) {
        detected_marker_id = markerIds[0];
        ROS_INFO("Detected marker ID: %d", detected_marker_id);
    } else {
        detected_marker_id = -1;
        ROS_INFO("No ArUco marker detected");
    }
}

// Service handler function
bool detectArucoMarkerService(aruco_detector::DetectArucoMarker::Request& req,
                              aruco_detector::DetectArucoMarker::Response& res) {
    if (!image_received) {
        ROS_WARN("No image available for marker detection.");
        res.marker_id = -1;
        return false;
    }

    // Process the image to detect ArUco markers
    detectArucoMarkerInImage(latest_image);

    // Set the service response with the detected marker ID or -1 if no marker is found
    if (detected_marker_id != -1) {
        res.marker_id = detected_marker_id;
    } else {
        res.marker_id = -1;  // No marker detected
    }

    // Reset image received flag for future service calls
    image_received = false;
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "aruco_detector");
    ros::NodeHandle nh;

    // Advertise the service
    ros::ServiceServer service = nh.advertiseService("detect_aruco_marker", detectArucoMarkerService);
    ROS_INFO("ArUco detector Service is ready.");

    // Subscribe to the image topic
    ros::Subscriber image_sub = nh.subscribe("/webcam/image_raw", 1, imageCallback);

    ros::spin();
    return 0;
}
