#include <ros/ros.h>
#include <ff_msgs/EkfState.h>
#include <string>
#include <ff_serial/serial.h>
#include <geometry_msgs/PoseStamped.h>

class PrinterController {
public:
    // Constructor
    // Inputs: nh - ROS NodeHandle for parameter loading and subscribing
    // Initializes serial communication, subscribes to EKF state updates,
    // opens serial port, sends initial printer commands.
    PrinterController(ros::NodeHandle& nh) : nh_(nh), ser_(std::bind(&PrinterController::serialReadCallback, this, std::placeholders::_1, std::placeholders::_2)) {
        nh_.param<std::string>("serial_port", serial_port_, "/dev/ttyACM0");
        nh_.param<int>("baud_rate", baud_rate_, 115200);

        ekf_sub_ = nh_.subscribe("honey/gnc/ekf", 1, &PrinterController::ekfCallback, this);

        ser_.SetTimeoutCallback(std::bind(&PrinterController::serialTimeoutCallback, this));
        ser_.SetShutdownCallback(std::bind(&PrinterController::serialShutdownCallback, this));

        if (ser_.Open(serial_port_, baud_rate_)) {
            ROS_INFO_STREAM("Serial port initialized.");
        } else {
            ROS_ERROR_STREAM("Failed to initialize serial port.");
        }

        sendPIDCommand(7.00, 0.10, 12.00, true);
        sendHomeCommand(0.00, 0.00, 0.00, true);
        stepPerUnit(88.89, 88.89, 400.00, 471.50, true);
        maxFeedrates(200.00, 200.00, 25.00, 25.00, true);
        maxAccel(7000, 6000, 100, 1000, true);
    }

    // Destructor
    // Closes the serial port when the object is destroyed
    ~PrinterController() {
        ser_.Close();
    }

    // Callback for EKF state updates
    // Inputs: msg - Pointer to the received EKF state message
    // Updates current pose, calculates pose difference, sends movement command to printer
    void ekfCallback(const ff_msgs::EkfState::ConstPtr& msg) {
        if (first_time) {
            pose_0.pose.position.x = msg->pose.position.x;
            pose_0.pose.position.y = msg->pose.position.y;
            pose_0.pose.position.z = msg->pose.position.z;
            first_time = false;
        }

        pose_current.pose.position.x = msg->pose.position.x;
        pose_current.pose.position.y = msg->pose.position.y;
        pose_current.pose.position.z = msg->pose.position.z;

        double diff_x = pose_current.pose.position.x - pose_0.pose.position.x;
        double diff_y = pose_current.pose.position.y - pose_0.pose.position.y;
        double diff_z = pose_current.pose.position.z - pose_0.pose.position.z;

        if (fabs(diff_x) < 0.023 && fabs(diff_y) < 0.023 && fabs(diff_z) < 0.023) {
            return;
        }

        pose_diff.pose.position.x = diff_x;
        pose_diff.pose.position.y = diff_y;
        pose_diff.pose.position.z = diff_z;

        std::string printer_cmd = "G1 X" + std::to_string(pose_diff.pose.position.x * 1000) + 
                                  " Y" + std::to_string(pose_diff.pose.position.y * 1000) +
                                  " Z" + std::to_string(pose_diff.pose.position.z * 1000) + "\n";
        ser_.Write(reinterpret_cast<const uint8_t*>(printer_cmd.c_str()), printer_cmd.length());
        ROS_INFO_STREAM("Sent G-code command to printer: " << printer_cmd);

        pose_0 = pose_current;
    }

    // Callback for serial read
    // Inputs: data - Pointer to the data received from serial port
    //         size - Size of the data received
    // Handles incoming data from the serial port
    void serialReadCallback(const uint8_t* data, size_t size) {
        // Implementation based on specific needs
    }

    // Callback for serial timeout
    // Reports a timeout event in serial communication
    void serialTimeoutCallback() {
        ROS_WARN_STREAM("Serial communication timeout occurred.");
    }

    // Callback for serial shutdown
    // Reports a shutdown event in serial communication
    void serialShutdownCallback() {
        ROS_ERROR_STREAM("Serial communication shutdown occurred.");
    }

    // Function to send PID command
    // Inputs: P - Proportional value for PID
    //         I - Integral value for PID
    //         D - Derivative value for PID
    //         isHotend - Flag indicating if the command is for the hotend (true) or not (false)
    // Sends PID tuning command to the printer
    void sendPIDCommand(double P, double I, double D, bool isHotend) {
        std::string pid_cmd;
        if (isHotend) {
            pid_cmd = "M301 P" + std::to_string(P) + " I" + std::to_string(I) + " D" + std::to_string(D) + "\n";
        }
        ser_.Write(reinterpret_cast<const uint8_t*>(pid_cmd.c_str()), pid_cmd.length());
        ROS_INFO_STREAM("Sent PID command to printer: " << pid_cmd);
    }

    // Function to send home command
    // Inputs: X - X axis value for home command
    //         Y - Y axis value for home command
    //         Z - Z axis value for home command
    //         isHotend - Flag indicating if the command is for the hotend (true) or not (false)
    // Sends home command to the printer
    void sendHomeCommand(double X, double Y, double Z, bool isHotend) {
        std::string home_cmd;
        if (isHotend) {
            home_cmd = "M206 X" + std::to_string(X) + " Y" + std::to_string(Y) + " Z" + std::to_string(Z) + "\n";
        }
        ser_.Write(reinterpret_cast<const uint8_t*>(home_cmd.c_str()), home_cmd.length());
    }

    // Function to set steps per unit
    // Inputs: X - Steps per unit for X axis
    //         Y - Steps per unit for Y axis
    //         Z - Steps per unit for Z axis
    //         E - Steps per unit for extruder axis
    //         isHotend - Flag indicating if the command is for the hotend (true) or not (false)
    // Sets steps per unit configuration on the printer
    void stepPerUnit(double X, double Y, double Z, double E, bool isHotend) {
        std::string step_cmd;
        if (isHotend) {
            step_cmd = "M92 X" + std::to_string(X) + " Y" + std::to_string(Y) + " Z" + std::to_string(Z) + " E" + std::to_string(E) + "\n";
        }
        ser_.Write(reinterpret_cast<const uint8_t*>(step_cmd.c_str()), step_cmd.length());
    }

    // Function to set max feedrates
    // Inputs: X - Maximum feedrate for X axis
    //         Y - Maximum feedrate for Y axis
    //         Z - Maximum feedrate for Z axis
    //         E - Maximum feedrate for extruder axis
    //         isHotend - Flag indicating if the command is for the hotend (true) or not (false)
    // Sets maximum feedrates configuration on the printer
    void maxFeedrates(double X, double Y, double Z, double E, bool isHotend) {
        std::string feedrates_cmd;
        if (isHotend) {
            feedrates_cmd = "M203 X" + std::to_string(X) + " Y" + std::to_string(Y) + " Z" + std::to_string(Z) + " E" + std::to_string(E) + "\n";
        }
        ser_.Write(reinterpret_cast<const uint8_t*>(feedrates_cmd.c_str()), feedrates_cmd.length());
    }

    // Function to set max acceleration
    // Inputs: X - Maximum acceleration for X axis
    //         Y - Maximum acceleration for Y axis
    //         Z - Maximum acceleration for Z axis
    //         E - Maximum acceleration for extruder axis
    //         isHotend - Flag indicating if the command is for the hotend (true) or not (false)
    // Sets maximum acceleration configuration on the printer
    void maxAccel(double X, double Y, double Z, double E, bool isHotend) {
        std::string accel_cmd;
        if (isHotend) {
            accel_cmd = "M201 X" + std::to_string(X) + " Y" + std::to_string(Y) + " Z" + std::to_string(Z) + " E" + std::to_string(E) + "\n";
        }
        ser_.Write(reinterpret_cast<const uint8_t*>(accel_cmd.c_str()), accel_cmd.length());
    }

    // Main loop
    // Keeps the ROS node spinning at a rate of 5 Hz
    void run() {
        ros::Rate rate(5);
        while (ros::ok()) {
            ros::spinOnce();
            rate.sleep
        }
    }