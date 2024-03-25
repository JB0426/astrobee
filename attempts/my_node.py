#!/usr/bin/env python

import rospy

from geometry_msgs.msg import PoseStamped, TwistStamped

from ff_msgs.msg import CommandStamped, CommandArg, EkfState, FamCommand


class MyNode(object): 

    def __init__(self):
        rospy.loginfo("My node is running") 
        pose_ts = 0.0
        twist_ts = 0.0

        pass
    

    def pose_sub_cb(self, msg=geometry_msgs.msg.PoseStamped()):
        """
        Pose callback to update the agent's position and attitude.

        :param msg: estimated pose, defaults to geometry_msgs.msg.PoseStamped()
        :type msg: geometry_msgs.msg.PoseStamped
        """

        self.pose_ts = rospy.get_time()
        self.pose = np.array([[msg.pose.position.x,
                               msg.pose.position.y,
                               msg.pose.position.z,
                               msg.pose.orientation.x,
                               msg.pose.orientation.y,
                               msg.pose.orientation.z,
                               msg.pose.orientation.w]]).T
        # Update state variable
        self.state[0:3] = self.pose[0:3]
        self.state[6:10] = self.pose[3:7]
        return

    def twist_sub_cb(self, msg=geometry_msgs.msg.TwistStamped()):
        """
        Twist callback to update the agent's linear and angular velocities.

        :param msg: estimated velocities
        :type msg: geometry_msgs.msg.TwistStamped
        """

        self.twist_ts = rospy.get_time()
        self.twist = np.array([[msg.twist.linear.x,
                                msg.twist.linear.y,
                                msg.twist.linear.z,
                                msg.twist.angular.x,
                                msg.twist.angular.y,
                                msg.twist.angular.z]]).T
        # Update state variable
        self.state[3:6] = self.twist[0:3]
        self.state[10:13] = self.twist[3:6]
        
        return
    
    def set_subscribers_publishers(self):
        """
        Helper function to create all publishers and subscribers.
        """

        # Subscribers
        self.pose_sub = rospy.Subscriber("~pose_topic",
                                         geometry_msgs.msg.PoseStamped,
                                         self.pose_sub_cb)
        self.twist_sub = rospy.Subscriber("~twist_topic",
                                          geometry_msgs.msg.TwistStamped,
                                          self.twist_sub_cb)
        
        # Publishers
        self.control_pub = rospy.Publisher("~control_topic",
                                           ff_msgs.msg.FamCommand,
                                           queue_size=1)
        
        pass


    def send_mobility_command():
        cmd = CommandStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.subsys_name = "Astrobee"
        # Initialize position to be the current position if not a relative move
        cmd.args[1].data_type = CommandArg.DATA_TYPE_VEC3d
        if FLAGS_relative:
            cmd.args[1].vec3d = [0, 0, 0]
        else:
            cmd.args[1].vec3d = [tfs.transform.translation.x,
                                 tfs.transform.translation.y,
                                 tfs.transform.translation.z]
    
        pass



    def  run(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('my_node')
    dmpc = MyNode()
    rospy.spin()
    pass
