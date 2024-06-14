#include <ros/ros.h>
#include <ff_msgs/EkfState.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/PoseStamped.h>

class ArduinoROS {
public:
    // Constructor
    // Inputs: nh - ROS NodeHandle for subscribing and advertising topics
    // Initializes subscribers for EKF state updates and publishers for stepper motor commands
    ArduinoROS(ros::NodeHandle& nh) : nh_(nh) {
        ekf_sub = nh_.subscribe("/honey/gnc/ekf", 10, &ArduinoROS::ekfCallback, this);

        arduino_pubx = nh_.advertise<std_msgs::Int8>("/stepperx", 1);
        arduino_puby = nh_.advertise<std_msgs::Int8>("/steppery", 1);
        arduino_pubz = nh_.advertise<std_msgs::Int8>("/stepperz", 1);
    }

    // Function to publish angle command to stepper motor X
    // Inputs: angx - Angle command for stepper motor X
    // Publishes the angle command to the topic "/stepperx"
    void stepperMotorX(int angx) {
        std_msgs::Int8 ang_cmd_x;
        ang_cmd_x.data = angx;
        arduino_pubx.publish(ang_cmd_x);
    }

    // Function to publish angle command to stepper motor Y
    // Inputs: angy - Angle command for stepper motor Y
    // Publishes the angle command to the topic "/steppery"
    void stepperMotorY(int angy) {
        std_msgs::Int8 ang_cmd_y;
        ang_cmd_y.data = angy;
        arduino_puby.publish(ang_cmd_y);
    }

    // Function to publish angle command to stepper motor Z
    // Inputs: angz - Angle command for stepper motor Z
    // Publishes the angle command to the topic "/stepperz"
    void stepperMotorZ(int angz) {
        std_msgs::Int8 ang_cmd_z;
        ang_cmd_z.data = angz;
        arduino_pubz.publish(ang_cmd_z);
    }

    // Callback function for EKF state updates
    // Inputs: msg - Pointer to the received EKF state message
    // Updates current and initial poses, calculates position differences,
    // converts differences to angles, and sends commands to stepper motors
    void ekfCallback(const ff_msgs::EkfState::ConstPtr& msg) {
        if (first_time) {
            pose_0 = msg->pose;
            first_time = false;
        }

        pose_current = msg->pose;

        double curr_diff_x = pose_current.position.x - pose_0.position.x;
        double curr_diff_y = pose_current.position.y - pose_0.position.y;
        double curr_diff_z = pose_current.position.z - pose_0.position.z;

        int angle_x = static_cast<int>(round(curr_diff_x * 150));
        stepperMotorX(angle_x);

        int angle_y = static_cast<int>(round(curr_diff_y * 250));
        stepperMotorY(angle_y);

        int angle_z = static_cast<int>(round(curr_diff_z * 360));
        stepperMotorZ(angle_z);

        pose_0 = pose_current;
    }

    // Main loop
    // Runs the ROS node, processing callbacks and maintaining a high frequency
    void run() {
        ros::Rate rate(20000); // Set the desired frequency (20 kHz)
        while (ros::ok()) {
            ros::spinOnce(); // Process callbacks
            rate.sleep();    // Sleep to maintain the desired frequency
        }
    }

private:
    ros::NodeHandle nh_;         // ROS node handle
    ros::Subscriber ekf_sub;     // Subscriber for EKF state updates
    ros::Publisher arduino_pubx; // Publisher for stepper motor X
    ros::Publisher arduino_puby; // Publisher for stepper motor Y
    ros::Publisher arduino_pubz; // Publisher for stepper motor Z
    bool first_time = true;      // Flag for the first EKF update
    geometry_msgs::Pose pose_0;  // Initial pose
    geometry_msgs::Pose pose_current; // Current pose
};

// Main function
int main(int argc, char** argv) {
    ros::init(argc, argv, "move_printer");
    ros::NodeHandle nh;
    ArduinoROS ard(nh);
    ard.run(); // Start the node
    return 0;
}
