#!/usr/bin/env python

import rospy
import time
import math
from ff_msgs.msg import EkfState, FamCommand, ControlCommand
from geometry_msgs.msg import PoseStamped, Vector3



first_time = True
pose_0 = PoseStamped()
pose_current = PoseStamped()

# Define desired position and orientation (adjust these values according to your requirements)
desired_position = Vector3(x=10.5, y=-0.5, z=4.8)
desired_orientation = Vector3(x=0.0, y=0.0, z=0.0)  # Desired orientation change in radians

# Proportional control gains for linear and angular control (adjust as needed)
kp_linear = 0.15
kp_angular = 0.2
distance_threshold = 0.2


rospy.init_node("bumble_auto_nav")
rate = rospy.Rate(80)
pub = rospy.Publisher("/bumble/gnc/ctl/command", FamCommand, queue_size=1)
#pub2 = rospy.Publisher("/cliked_point", ControlCommand, queue_size=1)


def callback(msg):
    global first_time, pose_0, pose_current
    if first_time:
        pose_0 = msg
        first_time = False
    pose_current = msg

rospy.Subscriber("/bumble/gnc/ekf", EkfState, callback)


def calculate_commands():
    global first_time, pose_0, pose_current, desired_position, desired_orientation, kp_angular, kp_linear, pub, distance_threshold
    # Calculate linear error (desired - current)
    error_linear = Vector3(
        x=desired_position.x - pose_current.pose.position.x,
        y=desired_position.y - pose_current.pose.position.y,
        z=desired_position.z - pose_current.pose.position.z
    )
    # Calculate angular error (desired - current)
    current_orientation = pose_current.pose.orientation
    error_orientation = Vector3(
        x=desired_orientation.x - current_orientation.x,
        y=desired_orientation.y - current_orientation.y,
        z=desired_orientation.z - current_orientation.z
    )
    distance_to_target = math.sqrt(error_linear.x ** 2 + error_linear.y ** 2 + error_linear.z ** 2)
    ctl_msg = ControlCommand()
    ctl_msg.header.stamp = rospy.Time.now()
    ctl_msg.header.frame_id = "world"
    cmd_msg = FamCommand()
    cmd_msg.header.stamp = rospy.Time.now()
    cmd_msg.header.frame_id = "body"

    if distance_to_target > distance_threshold:
        ctl_msg.mode = 2
        # Proportional control for linear motion
        cmd_msg.wrench.force.x = kp_linear * error_linear.x 
        cmd_msg.wrench.force.y = kp_linear * error_linear.y 
        cmd_msg.wrench.force.z = kp_linear * error_linear.z 
        
        # Proportional control for angular motion
        cmd_msg.wrench.torque.x = kp_angular * error_orientation.x 
        cmd_msg.wrench.torque.y = kp_angular * error_orientation.y 
        cmd_msg.wrench.torque.z = kp_angular * error_orientation.z

        #pub2.publish(ctl_msg)
        pub.publish(cmd_msg)
        
    else :
        ctl_msg.mode = 1
        #pub2.publish(ctl_msg)
    
    return cmd_msg
    #, ctl_msg
 
while not rospy.is_shutdown():
    cmd_msg = calculate_commands()
    #time.sleep(0.01)