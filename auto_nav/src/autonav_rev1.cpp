#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <ff_msgs/EkfState.h>
#include <ff_msgs/FamCommand.h>
#include <cmath>

// Include the control library
#include </home/iss-interns/astrobee/src/gnc/ctl/src/ctl_ros.cc>

class AutoNav 
{
public:
    // Constructor initializes the node handle and sets up publishers and subscribers
    AutoNav(ros::NodeHandle& nh) : nh_(nh), control(nh_, "control")
    {
        // Initialize publishers for control commands, trajectories, and segments
        ctl_pub = nh_.advertise<ff_msgs::FamCommand>("/honey/gnc/ctl/command", 1);
        traj_pub = nh_.advertise<ff_msgs::FamCommand>("/honey/gnc/ctl/traj", 1);
        segment_pub = nh_.advertise<ff_msgs::FamCommand>("/honey/gnc/ctl/segment", 1);

        // Initialize subscriber for EKF state updates
        ekf_sub = nh_.subscribe("/honey/gnc/ekf", 1, &AutoNav::ekfCallback, this);
        
        
        // goal_sub = nh_.subscribe("/honey/gnc/control/goal", 1, &AutoNav::goalCallback, this);
    }

    // Callback function for EKF state updates
    void ekfCallback(const ff_msgs::EkfState::ConstPtr& msg){
        control.EkfCallback(msg);
    }

    
    // void goalCallback(const ff_msgs::ControlGoal::ConstPtr& goal){
    //     control.GoalCallback(goal);
    // }

    // Function to perform a control step
    void ctlStep(ros::Time curr_time){
        control.Step(curr_time);
    }

    // Function to send a control command
    bool sendControlCommand(uint8_t mode, ff_msgs::ControlCommand poseVel){
        return control.Command(mode, poseVel);
    }

    // Function to get a control command
    float getControlCommand(ControlCommand* cmd, ros::Time time){
        return control.GetCommand(cmd, time);
    }

    // Main loop to continuously run the control steps
    void run() {
        ros::Rate rate(80); // Set the desired frequency (80 Hz)
        while (ros::ok()) {
            ros::spinOnce(); // Process callbacks
            rate.sleep();    // Sleep to maintain the desired frequency
        }
    }

private:
    ros::NodeHandle nh_;  // ROS node handle
    ros::Publisher ctl_pub;  // Publisher for control commands
    ros::Publisher traj_pub;  // Publisher for trajectories
    ros::Publisher segment_pub;  // Publisher for segments
    ros::Subscriber ekf_sub;  // Subscriber for EKF state updates
    ros::Subscriber goal_sub;  // Subscriber for goal updates
    Ctl control;  // Control object
};

// Main function
int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "auto_nav");
    ros::NodeHandle nh;
    AutoNav auto_nav(nh);

    // Perform an initial control step
    auto_nav.ctlStep(ros::Time::now());

    // Create a control command and send it
    ff_msgs::ControlCommand control_command;
    auto_nav.sendControlCommand(ff_msgs::ControlCommand::MODE_NOMINAL, control_command);

    // Get the current control command
    ff_msgs::ControlCommand cmd;
    float time_diff = auto_nav.getControlCommand(&cmd, ros::Time::now());

    // Start the main loop
    auto_nav.run(); 

    return 0;
}

