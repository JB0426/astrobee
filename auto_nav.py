#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from ff_msgs.msg import CommandStamped

def callback(msg):
    print(msg)

rospy.init_node("auto_nav")

pub = rospy.Publisher("/command", CommandStamped, queue_size = 1)
rospy.Subscriber("/loc/pose", PoseStamped, callback)
msg = CommandStamped()
msg.

while not rospy.is_shutdown():
    pub.publish(msg)
    rospy.sleep(1)

rospy.spin()