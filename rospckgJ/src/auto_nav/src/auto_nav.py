#!/usr/bin/env python

import rospy
#from geometry_msgs.msg import PoseStamped
from ff_msgs.msg import FamCommand
from ff_msgs.msg import ControlCommand

def callback(msg):
    print(msg)

rospy.init_node("auto_nav")

pub = rospy.Publisher("/honey/gnc/ctl/command", FamCommand, queue_size = 1)
#rospy.Subscriber("/loc/pose", PoseStamped, callback)
msg = FamCommand()
#control = ControlCommand()
#control.mode = 2
msg.wrench.force.y = 1.0
#msg.wrench.torque.z = 1.0


while not rospy.is_shutdown():
    pub.publish(msg)


#rospy.spin()S