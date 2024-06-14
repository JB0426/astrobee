#!/usr/bin/env python

import rospy
import time
import math
from ff_msgs.msg import EkfState, FamCommand
from geometry_msgs.msg import PoseStamped, Vector3

# Initialize global variables
first_time = True
pose_0 = PoseStamped()  # Initial pose
pose_current = PoseStamped()  # Current pose

# Define desired position and orientation (adjust these values according to your requirements)
desired_position = Vector3(x=11.0, y=-5.0, z=4.9)
desired_orientation = Vector3(x=0.0, y=0.0, z=0.0)  # Desired orientation change in radians
kp_linear = 0.2  # Proportional gain for linear control
kp_angular = 0.2  # Proportional gain for angular control

# Initialize the ROS node
rospy.init_node("auto_nav")

# Publisher for the FamCommand messages
pub = rospy.Publisher("/honey/gnc/ctl/command", FamCommand, queue_size=1)

# Callback function for the EKF state updates
# Inputs: msg (EkfState) - the current EKF state message
# Outputs: None
def callback(msg):
    global first_time, pose_0, pose_current
    if first_time:
        pose_0 = msg  # Set initial pose
        first_time = False
    pose_current = msg  # Update current pose

# Subscriber for the EKF state messages
rospy.Subscriber("/honey/gnc/ekf", EkfState, callback)

# Maximum velocity limit
max_velocity = 0.05

# Function to calculate control commands
# Inputs: None
# Outputs: cmd_msg (FamCommand) - the calculated command message
def calculate_commands():
    global pose_current, desired_position, desired_orientation, kp_angular, kp_linear, max_velocity

    # Calculate linear position error
    error_linear = Vector3(
        x=desired_position.x - pose_current.pose.position.x,
        y=desired_position.y - pose_current.pose.position.y,
        z=desired_position.z - pose_current.pose.position.z
    )

    # Calculate orientation error
    current_orientation = pose_current.pose.orientation
    error_orientation = Vector3(
        x=desired_orientation.x - current_orientation.x,
        y=desired_orientation.y - current_orientation.y,
        z=desired_orientation.z - current_orientation.z
    )

    # Calculate distance to the target position
    distance_to_target = math.sqrt(error_linear.x ** 2 + error_linear.y ** 2 + error_linear.z ** 2)

    # Initialize command message
    cmd_msg = FamCommand()
    cmd_msg.header.stamp = rospy.Time.now()
    cmd_msg.header.frame_id = "body"
    cmd_msg.control_mode = 2  # Use force/torque control mode
    
    max_force = 0.085  # Maximum force to be applied

    if distance_to_target > 0.6:
        # Calculate force magnitude and desired velocity
        force_magnitude = min(kp_linear * distance_to_target, max_force)
        desired_velocity = min(max_velocity, distance_to_target * kp_linear)
     
        # Calculate direction vector
        direction = Vector3(
            x=error_linear.x / distance_to_target,
            y=error_linear.y / distance_to_target,
            z=error_linear.z / distance_to_target
        )
        
        # Apply the calculated force and velocity in the direction of the error
        cmd_msg.wrench.force.x = min(direction.x * force_magnitude, desired_velocity)
        cmd_msg.wrench.force.y = min(direction.y * force_magnitude, desired_velocity)
        cmd_msg.wrench.force.z = min(direction.z * force_magnitude, desired_velocity)
        
        # Apply calculated torque for orientation adjustment
        cmd_msg.wrench.torque.x = kp_angular * error_orientation.x * 0.1
        cmd_msg.wrench.torque.y = kp_angular * error_orientation.y * 0.1
        cmd_msg.wrench.torque.z = kp_angular * error_orientation.z * 0.1
    else:
        cmd_msg.control_mode = 1  # Switch to position hold mode if close to target

    pub.publish(cmd_msg)  # Publish the command

# Main loop to continuously calculate and publish commands
# Inputs: None
# Outputs: None
while not rospy.is_shutdown():
    cmd_msg = calculate_commands()  # Calculate new commands
    time.sleep(0.01)  # Sleep for 10ms before next iteration


