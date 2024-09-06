#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Bool.h>
#include <math.h>

mavros_msgs::State current_state; // Global variable
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

geometry_msgs::PoseStamped current_pose; // Global variable
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_pose = *msg;
}

bool obstacle_detected = false;  // Global variable to store obstacle state
void obstacle_cb(const std_msgs::Bool::ConstPtr& msg) {
    obstacle_detected = msg->data;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    // Subscribers
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, pose_cb);
    ros::Subscriber obstacle_sub = nh.subscribe<std_msgs::Bool>
            ("/obstacle_detected", 10, obstacle_cb);  // Subscribe to obstacle detection topic

    // Publishers and service clients
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::Publisher velocity_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    // Setpoint publishing rate
    ros::Rate rate(20.0);

    // Wait for FCU connection
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    // Initial setpoint
    geometry_msgs::PoseStamped setpoint;
    setpoint.pose.position.x = 0;
    setpoint.pose.position.y = 0;
    setpoint.pose.position.z = 2;

    ROS_INFO("First Setpoint: [0, 0, 2]");

    float x, y, z;
    float prev_x = setpoint.pose.position.x;
    float prev_y = setpoint.pose.position.y;
    float prev_z = setpoint.pose.position.z;
    float tolerance = 0.1;
    bool velocity_control_active = false;
    float velocity = 1.0;

    // Send a few initial setpoints before starting
    for (int i = 100; ros::ok() && i > 0; --i) {
        local_pos_pub.publish(setpoint);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while (ros::ok()) {
        // Enable offboard mode and arm the vehicle if not already done
        if (current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if (set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if (!current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))) {
                if (arming_client.call(arm_cmd) &&
                    arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        // Check if an obstacle is detected
        if (obstacle_detected) {
            ROS_WARN("Obstacle detected!");
            // Stop UAV by publishing zero velocity
            geometry_msgs::TwistStamped stop_vel_cmd;
            stop_vel_cmd.twist.linear.x = 0.0;
            stop_vel_cmd.twist.linear.y = 0.0;
            stop_vel_cmd.twist.linear.z = 0.0; // Update velocity to 0 in all directions
            velocity_pub.publish(stop_vel_cmd);

            // Set new setpoint above the current position
            geometry_msgs::PoseStamped new_setpoint_obstacle = current_pose;
            new_setpoint_obstacle.pose.position.z += 0.1;

            local_pos_pub.publish(new_setpoint_obstacle); // Publish the new setpoint

        } else {

            ROS_INFO_THROTTLE(5, "No obstacle detected");

            // Normal control logic when no obstacle is detected
            if (nh.getParam("uav_setpoint/x", x) && nh.getParam("uav_setpoint/y", y) && nh.getParam("uav_setpoint/z", z)) {
                setpoint.pose.position.x = x;
                setpoint.pose.position.y = y;
                setpoint.pose.position.z = z;

                if (setpoint.pose.position.x != prev_x || setpoint.pose.position.y != prev_y || setpoint.pose.position.z != prev_z) {
                    ROS_INFO("New UAV Setpoint is: [%f, %f, %f]",
                             setpoint.pose.position.x, setpoint.pose.position.y, setpoint.pose.position.z);

                    prev_x = setpoint.pose.position.x;
                    prev_y = setpoint.pose.position.y;
                    prev_z = setpoint.pose.position.z;

                    velocity_control_active = true;
                    ROS_INFO("Switching to velocity control.");
                }
            }

            if (velocity_control_active) {
                float dx = setpoint.pose.position.x - current_pose.pose.position.x;
                float dy = setpoint.pose.position.y - current_pose.pose.position.y;
                float dz = setpoint.pose.position.z - current_pose.pose.position.z;

                float distance_to_setpoint = sqrt(dx * dx + dy * dy + dz * dz);

                if (distance_to_setpoint <= tolerance) {
                    ROS_INFO("Reached target in velocity control. Switching to position control.");

                    geometry_msgs::TwistStamped stop_vel_cmd;
                    stop_vel_cmd.twist.linear.x = 0.0;
                    stop_vel_cmd.twist.linear.y = 0.0;
                    stop_vel_cmd.twist.linear.z = 0.0;
                    velocity_pub.publish(stop_vel_cmd);

                    velocity_control_active = false;
                    local_pos_pub.publish(setpoint);

                } else {
                    geometry_msgs::TwistStamped vel_cmd;
                    vel_cmd.twist.linear.x = (dx / distance_to_setpoint) * velocity;
                    vel_cmd.twist.linear.y = (dy / distance_to_setpoint) * velocity;
                    vel_cmd.twist.linear.z = (dz / distance_to_setpoint) * velocity;

                    velocity_pub.publish(vel_cmd);
                }

            } else { // precise positioning using position control instead of velocity
                local_pos_pub.publish(setpoint);
            }

        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
