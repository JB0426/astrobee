#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

# Node initialization
rospy.init_node('init_pose')
pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)

# Construct message
init_msg = PoseWithCovarianceStamped()
init_msg.header.frame_id = "rviz"

# Get initial pose from Gazebo
loc_pose_msg = rospy.wait_for_message('/odom',Odometry)
init_msg.pose.pose.position.x = loc_pose_msg.pose.position.x
init_msg.pose.pose.position.y = loc_pose_msg.pose.position.y
init_msg.pose.pose.position.z = loc_pose_msg.pose.position.z
init_msg.pose.pose.orientation.x = loc_pose_msg.pose.orientation.x
init_msg.pose.pose.orientation.y = loc_pose_msg.pose.orientation.y
init_msg.pose.pose.orientation.z = loc_pose_msg.pose.orientation.z
init_msg.pose.pose.orientation.w = loc_pose_msg.pose.orientation.w

# Delay
rospy.sleep(1)

# Publish message
rospy.loginfo("setting initial pose")
pub.publish(init_msg)
rospy.loginfo("initial pose is set")