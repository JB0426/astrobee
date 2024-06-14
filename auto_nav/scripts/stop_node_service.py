#!/usr/bin/env python

import rospy
import os
from auto_nav.srv import StopNode

# Handler function for the stop node service
def handle_stop_node(req):
    try:
        # Use os.system to kill the specified ROS node
        os.system("rosnode kill /honey/ctl")
        rospy.loginfo("Stopped /honey/ctl node")
        return {'success': True}
    except Exception as e:
        rospy.logerr("Failed to stop /honey/ctl node: %s", str(e))
        return {'success': False}

# Function to set up and start the stop node service
def stop_node_service():
    # Initialize the ROS node
    rospy.init_node('stop_node_service')
    # Define the stop_node service and link it to the handler function
    rospy.Service('stop_node', StopNode, handle_stop_node)
    rospy.loginfo("Stop node service ready.")
    # Keep the service running
    rospy.spin()

# Main function
if __name__ == "__main__":
    stop_node_service()
