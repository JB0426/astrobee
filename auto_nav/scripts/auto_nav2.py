#!/usr/bin/env python

import rospy
import math
from ff_msgs.msg import EkfState, FamCommand
from geometry_msgs.msg import PoseStamped, Vector3


first_time = True
pose_0 = PoseStamped()
pose_current = PoseStamped()

# Define desired position and orientation (adjust these values according to your requirements)
desired_position = Vector3(x=11.0, y=-4.0, z=4.8)
desired_orientation = Vector3(x=0.0, y=0.0, z=0.0)  # Desired orientation change in radians

# Proportional control gains for linear and angular control (adjust as needed)
kp_linear = 0.3
kp_angular = 0.3

# Distance threshold for braking
distance_threshold = 1


def callback(msg):
    global first_time, pose_0, pose_current

    if first_time:
        pose_0 = msg
        first_time = False
    
    pose_current = msg

def calculate_commands():
    # Calculate linear error (desired - current)
    error_linear = Vector3(
        x=desired_position.x - pose_current.pose.position.x,
        y=desired_position.y - pose_current.pose.position.y,
        z=desired_position.z - pose_current.pose.position.z
    )

    # Proportional control for linear motion
    command_msg = FamCommand()
    command_msg.wrench.force.x = kp_linear * error_linear.x 
    command_msg.wrench.force.y = kp_linear * error_linear.y 
    command_msg.wrench.force.z = kp_linear * error_linear.z 

      # Apply braking if the distance to the desired point is below the threshold
    distance_to_target = math.sqrt(error_linear.x ** 2 + error_linear.y ** 2 + error_linear.z ** 2)
    if distance_to_target < distance_threshold:
        # Calculate braking force (opposite direction of current velocity)
        braking_force = Vector3(
            x=-kp_linear * pose_current.accel.x,
            y=-kp_linear * pose_current.accel.y,
            z=-kp_linear * pose_current.accel.z
        )
        # Apply braking force
        command_msg.wrench.force.x = braking_force.x 
        command_msg.wrench.force.y = braking_force.y 
        command_msg.wrench.force.z = braking_force.z 



    # Calculate angular error (desired - current)
    current_orientation = pose_current.pose.orientation
    error_orientation = Vector3(
        x=desired_orientation.x - current_orientation.x,
        y=desired_orientation.y - current_orientation.y,
        z=desired_orientation.z - current_orientation.z
    )

    # Proportional control for angular motion
    command_msg.wrench.torque.x = kp_angular * error_orientation.x
    command_msg.wrench.torque.y = kp_angular * error_orientation.y
    command_msg.wrench.torque.z = kp_angular * error_orientation.z

    return command_msg

rospy.init_node("auto_nav")
pub = rospy.Publisher("/honey/gnc/ctl/command", FamCommand, queue_size=1)
rospy.Subscriber("/honey/gnc/ekf", EkfState, callback)

rospy.sleep(1.0 / 60)  # Control loop rate of 60 Hz

while not rospy.is_shutdown():
    command_msg = calculate_commands()
    pub.publish(command_msg)
  
rospy.spin()