#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <ff_msgs/EkfState.h>
#include <ff_msgs/FamCommand.h>
#include <cmath>

class AutoNav 
{
public:
    // Constructor
    AutoNav(ros::NodeHandle& nh) : nh_(nh), first_time(true), kp_linear(0.2), kp_angular(0.2), max_velocity(0.05), max_force(0.085)
    {
        // Initialize desired position and orientation
        desired_position.x = 11.0;
        desired_position.y = -4.0;
        desired_position.z = 4.9;

        desired_orientation.x = 0.0;
        desired_orientation.y = 0.0;
        desired_orientation.z = 0.0;

        // Set up publisher and subscriber
        pub = nh_.advertise<ff_msgs::FamCommand>("/honey/gnc/ctl/command", 1);
        sub = nh_.subscribe("/honey/gnc/ekf", 1, &AutoNav::callback, this);
    }

    // Function to calculate control commands
    void calculate_commands() {
        // Calculate position errors
        geometry_msgs::Vector3 error_linear;
        error_linear.x = desired_position.x - pose_current.pose.position.x;
        error_linear.y = desired_position.y - pose_current.pose.position.y;
        error_linear.z = desired_position.z - pose_current.pose.position.z;

        // Calculate orientation errors
        geometry_msgs::Vector3 error_orientation;
        error_orientation.x = desired_orientation.x - pose_current.pose.orientation.x;
        error_orientation.y = desired_orientation.y - pose_current.pose.orientation.y;
        error_orientation.z = desired_orientation.z - pose_current.pose.orientation.z;

        // Calculate distance to target
        double distance_to_target = sqrt(pow(error_linear.x, 2) + pow(error_linear.y, 2) + pow(error_linear.z, 2));

        // Create FAM command message
        ff_msgs::FamCommand cmd_msg;
        cmd_msg.header.stamp = ros::Time::now();
        cmd_msg.header.frame_id = "body";
        cmd_msg.control_mode = 2;

        if (distance_to_target > 0.6) {
            // Calculate force magnitude and desired velocity
            double force_magnitude = std::min(kp_linear * distance_to_target, max_force);
            double desired_velocity = std::min(max_velocity, distance_to_target * kp_linear);

            // Calculate direction of the force
            geometry_msgs::Vector3 direction;
            direction.x = error_linear.x / distance_to_target;
            direction.y = error_linear.y / distance_to_target;
            direction.z = error_linear.z / distance_to_target;

            // Set force and torque commands
            cmd_msg.wrench.force.x = std::min(direction.x * force_magnitude, desired_velocity);
            cmd_msg.wrench.force.y = std::min(direction.y * force_magnitude, desired_velocity);
            cmd_msg.wrench.force.z = std::min(direction.z * force_magnitude, desired_velocity);

            cmd_msg.wrench.torque.x = kp_angular * error_orientation.x * 0.1;
            cmd_msg.wrench.torque.y = kp_angular * error_orientation.y * 0.1;
            cmd_msg.wrench.torque.z = kp_angular * error_orientation.z * 0.1;
        } else {
            // Switch to control mode 1 if close to the target
            cmd_msg.control_mode = 1;
        }

        // Publish the command message
        pub.publish(cmd_msg);
    }

    // Callback function for EKF state updates
    // Inputs: msg (const ff_msgs::EkfState::ConstPtr&) - the received EKF state message
    // Outputs: None
    void callback(const ff_msgs::EkfState::ConstPtr& msg) {
        if (first_time) {
            // Store the initial pose
            pose_0.pose.position.x = msg->pose.position.x;
            pose_0.pose.position.y = msg->pose.position.y;
            pose_0.pose.position.z = msg->pose.position.z;
            first_time = false;
        }

        // Update current pose
        pose_current.pose.position.x = msg->pose.position.x;
        pose_current.pose.position.y = msg->pose.position.y;
        pose_current.pose.position.z = msg->pose.position.z;

        // Log current pose
        ROS_INFO("Pose current (y): (%f)", pose_current.pose.position.y);

        // Calculate and publish commands
        calculate_commands();
    }

    // Main loop
    // Inputs: argc (int) - number of command-line arguments
    //         argv (char**) - array of command-line arguments
    // Outputs: int - 0 for success
    void run() {
        ros::Rate rate(80); // Set the desired frequency (80 Hz)
        while (ros::ok()) {
            ros::spinOnce(); // Process callbacks
            rate.sleep();    // Sleep to maintain the desired frequency
        }
    }

private:
    ros::NodeHandle nh_;         // ROS node handle
    ros::Publisher pub;          // Publisher for FAM commands
    ros::Subscriber sub;         // Subscriber for EKF state updates
    bool first_time;             // Flag for the first EKF update
    geometry_msgs::PoseStamped pose_0; // Initial pose
    geometry_msgs::PoseStamped pose_current; // Current pose
    geometry_msgs::Vector3 desired_position; // Desired position
    geometry_msgs::Vector3 desired_orientation; // Desired orientation
    double kp_linear;            // Linear proportional gain
    double kp_angular;           // Angular proportional gain
    double max_velocity;         // Maximum velocity
    double max_force;            // Maximum force
};

// Main function
int main(int argc, char** argv) {
    ros::init(argc, argv, "auto_nav");
    ros::NodeHandle nh;
    AutoNav auto_nav(nh);
    auto_nav.run(); // Start the node
    return 0;
}
