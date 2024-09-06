#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Float64.h>
#include "link_attacher/AttachDetach.h" // Include the custom service message header
#include "aruco_detector/DetectArucoMarker.h"  // Include the custom service message header
#include <math.h>
#include <map>
#include <string>
#include <yaml-cpp/yaml.h>

// Enum to represent different states of the mission
enum class UAVState {
    WAITING_FOR_SETPOINT,
    TAKEOFF,
    NAVIGATE_TO_SETPOINT_VELOCITY_CONTROL,
    NAVIGATE_TO_SETPOINT_POSITION_CONTROL,
    LAND_AT_SETPOINT,
    MARKER_DETECTION,
    PICK_THE_OBJECT,
    TAKEOFF_FROM_SETPOINT,
    RETURN_TO_START_VELOCITY_CONTROL,
    RETURN_TO_START_POSITION_CONTROL,
    FINAL_LAND,
    PLACE_THE_OBJECT
};

// Global variables
mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;

// Callback functions
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_pose = *msg;
}

// Function to load marker map from YAML file
std::map<int, std::string> loadMarkerMap(const std::string& file_path) {
    std::map<int, std::string> markerMap;
    YAML::Node config = YAML::LoadFile(file_path);
    // Parse the "markers" node
    if (config["markers"]) {
        YAML::Node markers = config["markers"];
        for (const auto& item : markers) {
            int key = item.first.as<int>();
            std::string value = item.second.as<std::string>();
            markerMap[key] = value;
        }
    } else {
        ROS_ERROR("No markers found in YAML file!");
    }
    return markerMap;
}

// Function to update setpoint from ROS parameters
bool checkForNewSetpoint(ros::NodeHandle &nh, geometry_msgs::PoseStamped &setpoint_pose) {
    float x, y, z;
    if (nh.getParam("uav_setpoint/x", x) && nh.getParam("uav_setpoint/y", y) && nh.getParam("uav_setpoint/z", z)) {
        setpoint_pose.pose.position.x = x;
        setpoint_pose.pose.position.y = y;
        setpoint_pose.pose.position.z = z;
        ROS_INFO("New setpoint received: [%f, %f, %f]", x, y, z);
        // Delete the parameters after reading
        nh.deleteParam("uav_setpoint/x");
        nh.deleteParam("uav_setpoint/y");
        nh.deleteParam("uav_setpoint/z");
        return true;
    }
    return false;
}

// Function to check if UAV has reached the desired position
bool reachedTargetPosition(const geometry_msgs::PoseStamped& target_pose, UAVState mode, double xy_tolerance = 0.1, double z_tolerance = 0.3) {
    double dx = target_pose.pose.position.x - current_pose.pose.position.x;
    double dy = target_pose.pose.position.y - current_pose.pose.position.y;
    double dz = target_pose.pose.position.z - current_pose.pose.position.z;
    // Check xy-plane tolerance
    bool xy_reached = std::sqrt(dx * dx + dy * dy) < xy_tolerance;
    // Check z-plane tolerance
    bool z_reached = std::fabs(dz) < z_tolerance;
    if (xy_reached && z_reached) {
        if (mode == UAVState::NAVIGATE_TO_SETPOINT_VELOCITY_CONTROL) {
            ROS_INFO("Reached target position in Velocity Control Mode");
        } else {
            ROS_INFO("Reached target position in Position Control Mode");
        }
        return true;
    } else {
        return false;
    }
}

// Function to calculate the velocity command based on the current position and setpoint
geometry_msgs::TwistStamped handleVelocityControl(const geometry_msgs::PoseStamped &current_pose, const geometry_msgs::PoseStamped &setpoint, float velocity) {
    geometry_msgs::TwistStamped vel_cmd;
    float dx = setpoint.pose.position.x - current_pose.pose.position.x;
    float dy = setpoint.pose.position.y - current_pose.pose.position.y;
    float dz = setpoint.pose.position.z - current_pose.pose.position.z;
    float distance_to_setpoint = sqrt(dx * dx + dy * dy + dz * dz);
    // Calculate the velocity command towards the setpoint
    vel_cmd.twist.linear.x = (dx / distance_to_setpoint) * velocity;
    vel_cmd.twist.linear.y = (dy / distance_to_setpoint) * velocity;
    vel_cmd.twist.linear.z = (dz / distance_to_setpoint) * velocity;
    return vel_cmd;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "uav_pick_and_place");
    ros::NodeHandle nh;

    // Subscribers
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, pose_cb);

    // Publishers
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::Publisher velocity_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);

    // Services
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::ServiceClient attach_client = nh.serviceClient<link_attacher::AttachDetach>("start_link_attachment");
    ros::ServiceClient detach_client = nh.serviceClient<link_attacher::AttachDetach>("start_link_detachment");
    ros::ServiceClient aruco_client = nh.serviceClient<aruco_detector::DetectArucoMarker>("detect_aruco_marker");

    // Setpoint publishing rate
    ros::Rate rate(20.0);

    // Wait for FCU connection
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    bool state_changed = true;  // Flag to track state changes
    bool mission_started = false; // Flag to indicate if the mission has started

    geometry_msgs::PoseStamped start_pose;
    start_pose.pose.position.x = 0;
    start_pose.pose.position.y = 0;
    start_pose.pose.position.z = 2;
    geometry_msgs::PoseStamped setpoint_pose;
    geometry_msgs::PoseStamped landing_pose;
    geometry_msgs::PoseStamped takeoff_pose;
    geometry_msgs::PoseStamped final_landing_pose;
    float velocity = 1.0;

    // Load the marker map from the YAML file
    std::map<int, std::string> markerMap = loadMarkerMap(
        "/home/musta/catkin_ws/src/offb/config/id_to_object.yaml");
    int detected_marker_id;
    std::string marker_model_name;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(start_pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    UAVState uav_state = UAVState::WAITING_FOR_SETPOINT;

    while (ros::ok()) {

        // Continuously try to enable offboard mode and arm the vehicle
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        // Check for new setpoint if mission is not started
        if (!mission_started) {
            if (checkForNewSetpoint(nh, setpoint_pose)) {
                mission_started = true;
                uav_state = UAVState::NAVIGATE_TO_SETPOINT_VELOCITY_CONTROL;
                ROS_INFO("Setpoint received. Mission started.");
            }
        }

        // Execute state machine if mission has started
        if (mission_started) {
            switch (uav_state) {

                case UAVState::WAITING_FOR_SETPOINT: {
                    if (state_changed) {ROS_INFO("%sWAITING_FOR_SETPOINT%s","\033[1;32m","\033[0m");}
                    state_changed = false;
                    local_pos_pub.publish(start_pose);
                    if (checkForNewSetpoint(nh, setpoint_pose)) {
                        ROS_INFO("Setpoint received. Starting new mission.");
                        uav_state = UAVState::NAVIGATE_TO_SETPOINT_VELOCITY_CONTROL;
                        state_changed = true;
                    }
                    break;
                }

                case UAVState::TAKEOFF: {
                    if (state_changed) {ROS_INFO("%sTAKEOFF%s","\033[1;32m","\033[0m");}
                    state_changed = false;
                    local_pos_pub.publish(start_pose);
                    if (reachedTargetPosition(start_pose, uav_state)) {
                        uav_state = UAVState::NAVIGATE_TO_SETPOINT_VELOCITY_CONTROL;
                        state_changed = true;
                    }
                    break;
                }

                case UAVState::NAVIGATE_TO_SETPOINT_VELOCITY_CONTROL: {
                    if (state_changed) {ROS_INFO("%sNAVIGATE_TO_SETPOINT_VELOCITY_CONTROL%s","\033[1;32m","\033[0m");}
                    state_changed = false;
                    geometry_msgs::TwistStamped vel_cmd = handleVelocityControl(
                        current_pose, setpoint_pose, velocity);
                    velocity_pub.publish(vel_cmd);
                    if (reachedTargetPosition(setpoint_pose, uav_state, 0.3, 0.3)) {
                        uav_state = UAVState::NAVIGATE_TO_SETPOINT_POSITION_CONTROL;
                        state_changed = true;
                    }
                    break;
                }

                case UAVState::NAVIGATE_TO_SETPOINT_POSITION_CONTROL: {
                    if (state_changed) {ROS_INFO("%sNAVIGATE_TO_SETPOINT_POSITION_CONTROL%s","\033[1;32m","\033[0m");}
                    state_changed = false;
                    local_pos_pub.publish(setpoint_pose);
                    if (reachedTargetPosition(setpoint_pose, uav_state)) {
                        uav_state = UAVState::LAND_AT_SETPOINT;
                        state_changed = true;
                        landing_pose = setpoint_pose;
                    }
                    break;
                }

                case UAVState::LAND_AT_SETPOINT: {
                    if (state_changed) {ROS_INFO("%sLAND_AT_SETPOINT%s","\033[1;32m","\033[0m");}
                    state_changed = false;
                    if (landing_pose.pose.position.z > -2.0) {
                        landing_pose.pose.position.z -= 0.01;  // Decrease by 0.01 meters per loop iteration
                    }
                    local_pos_pub.publish(landing_pose);
                    final_landing_pose.pose.position.x = setpoint_pose.pose.position.x;
                    final_landing_pose.pose.position.y = setpoint_pose.pose.position.y;
                    final_landing_pose.pose.position.z = 0.0;
                    if (reachedTargetPosition(final_landing_pose, uav_state, 0.1, 0.01)) {
                        ROS_INFO("Landing completed at setpoint. Detecting AruCo Marker.");
                        uav_state = UAVState::MARKER_DETECTION;
                        state_changed = true;
                    }
                    break;
                }

                case UAVState::MARKER_DETECTION: {
                    if (state_changed) {ROS_INFO("%sMARKER_DETECTION%s","\033[1;32m","\033[0m");}
                    state_changed = false;
                    local_pos_pub.publish(landing_pose);
                    aruco_detector::DetectArucoMarker aruco_srv;
                    if (aruco_client.call(aruco_srv)) {
                        if (aruco_srv.response.marker_id != -1) {
                            ROS_INFO("Detected ArUco marker ID: %d", aruco_srv.response.marker_id);
                            detected_marker_id = aruco_srv.response.marker_id;
                            marker_model_name = markerMap[detected_marker_id];
                            ROS_INFO("Corresponding link name: %s", marker_model_name.c_str());
                            uav_state = UAVState::PICK_THE_OBJECT;
                        } else {
                            ROS_WARN("No marker detected. No Object to pick.");
                            uav_state = UAVState::TAKEOFF_FROM_SETPOINT;
                        }
                    } else {
                        ROS_ERROR("Failed to call /detect_aruco_marker service");
                        uav_state = UAVState::TAKEOFF_FROM_SETPOINT;
                    }
                    state_changed = true;
                    break;
                }

                case UAVState::PICK_THE_OBJECT: {
                    if (state_changed) {ROS_INFO("%sPICK_THE_OBJECT%s","\033[1;32m","\033[0m");}
                    state_changed = false;
                    local_pos_pub.publish(landing_pose);
                    link_attacher::AttachDetach attach_srv;
                    attach_srv.request.model_name_1 = "iris";
                    attach_srv.request.link_name_1 = "iris::base_link";
                    attach_srv.request.model_name_2 = marker_model_name;
                    attach_srv.request.link_name_2 = marker_model_name+"::small_box::box_link";
                    if (attach_client.call(attach_srv)) {
                        ROS_INFO("Successfully called /start_link_attachment service.");
                    } else { 
                        ROS_ERROR("Failed to call /start_link_attachment service.");  
                    }
                    uav_state = UAVState::TAKEOFF_FROM_SETPOINT;
                    takeoff_pose = setpoint_pose;
                    state_changed = true;
                    break;
                }

                case UAVState::TAKEOFF_FROM_SETPOINT: {
                    if (state_changed) {ROS_INFO("%sTAKEOFF_FROM_SETPOINT%s","\033[1;32m","\033[0m");}
                    state_changed = false;
                    if (takeoff_pose.pose.position.z < setpoint_pose.pose.position.z) {
                        takeoff_pose.pose.position.z += 0.1;  // Increase by 0.1 meters per loop iteration
                    }
                    local_pos_pub.publish(takeoff_pose);
                    if (reachedTargetPosition(setpoint_pose, uav_state, 0.1, 0.1)) {
                        uav_state = UAVState::RETURN_TO_START_VELOCITY_CONTROL;
                        state_changed = true;
                    }
                    break;
                }

                case UAVState::RETURN_TO_START_VELOCITY_CONTROL: {
                    if (state_changed) {ROS_INFO("%sRETURN_TO_START_VELOCITY_CONTROL%s","\033[1;32m","\033[0m");}
                    state_changed = false;
                    geometry_msgs::TwistStamped vel_cmd = handleVelocityControl(
                        current_pose, start_pose, velocity);
                    velocity_pub.publish(vel_cmd);
                    if (reachedTargetPosition(start_pose, uav_state, 0.3, 0.3)) {
                        uav_state = UAVState::RETURN_TO_START_POSITION_CONTROL;
                        state_changed = true;
                    }
                    break;
                }

                case UAVState::RETURN_TO_START_POSITION_CONTROL: {
                    if (state_changed) {ROS_INFO("%sRETURN_TO_START_POSITION_CONTROL%s","\033[1;32m","\033[0m");}
                    state_changed = false;
                    local_pos_pub.publish(start_pose);
                    if (reachedTargetPosition(start_pose, uav_state)) {
                        uav_state = UAVState::FINAL_LAND;
                        state_changed = true;
                        landing_pose = start_pose;
                    }
                    break;
                }

                case UAVState::FINAL_LAND: {
                    if (state_changed) {ROS_INFO("%sFINAL_LAND%s","\033[1;32m","\033[0m");}
                    state_changed = false;
                    if (landing_pose.pose.position.z > -2.0) {
                        landing_pose.pose.position.z -= 0.01;  // Decrease by 0.01 meters per loop iteration
                    }
                    local_pos_pub.publish(landing_pose);
                    final_landing_pose.pose.position.x = start_pose.pose.position.x;
                    final_landing_pose.pose.position.y = start_pose.pose.position.y;
                    final_landing_pose.pose.position.z = 1.0;
                    if (reachedTargetPosition(final_landing_pose, uav_state, 0.1, 0.01)) {
                        ROS_INFO("Landing completed at starting position. Placing the object");
                        uav_state = UAVState::PLACE_THE_OBJECT;
                        state_changed = true;
                    }
                    break;
                }

                case UAVState::PLACE_THE_OBJECT: {
                    if (state_changed) {ROS_INFO("%sPLACE_THE_OBJECT%s","\033[1;32m","\033[0m");}
                    state_changed = false;
                    local_pos_pub.publish(start_pose);
                    link_attacher::AttachDetach detach_srv;
                    detach_srv.request.model_name_1 = "iris";
                    detach_srv.request.link_name_1 = "iris::base_link";
                    detach_srv.request.model_name_2 = marker_model_name;
                    detach_srv.request.link_name_2 = marker_model_name+"::small_box::box_link";
                    if (detach_client.call(detach_srv)) {
                        ROS_INFO("Successfully called /start_link_detachment service.");
                    } else { 
                        ROS_ERROR("Failed to call /start_link_detachment service.");  
                    }
                    uav_state = UAVState::WAITING_FOR_SETPOINT;
                    state_changed = true;
                    break;
                }
            }

        } else {
            local_pos_pub.publish(start_pose);
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
