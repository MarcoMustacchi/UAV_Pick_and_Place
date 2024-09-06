#include <ros/ros.h>
#include <gazebo_ros_link_attacher/Attach.h>
#include "link_attacher/AttachDetach.h" // Custom service message

// Service callback function
bool handleAttachRequest(link_attacher::AttachDetach::Request &req, link_attacher::AttachDetach::Response &res) {
    // Extract model and link names from the request
    std::string model_name_1 = req.model_name_1;
    std::string link_name_1 = req.link_name_1;
    std::string model_name_2 = req.model_name_2;
    std::string link_name_2 = req.link_name_2;

    // Create a ROS node handle
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<gazebo_ros_link_attacher::Attach>("link_attacher_node/attach");

    // Prepare the attach service request
    gazebo_ros_link_attacher::Attach srv;
    srv.request.model_name_1 = model_name_1;
    srv.request.link_name_1 = link_name_1;
    srv.request.model_name_2 = model_name_2;
    srv.request.link_name_2 = link_name_2;

    // Call the attach service
    if (client.call(srv)) {
        ROS_INFO("Successfully attached %s to %s", link_name_1.c_str(), link_name_2.c_str());
        return true;
    } else {
        ROS_ERROR("Failed to call service link_attacher_node/attach");
        return false;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "link_attacher_service_node");
    ros::NodeHandle nh;

    // Advertise the service that will trigger the attachment
    ros::ServiceServer service = nh.advertiseService("start_link_attachment", handleAttachRequest);
    ROS_INFO("Link Attacher Service is ready.");

    ros::spin();
    return 0;
}

