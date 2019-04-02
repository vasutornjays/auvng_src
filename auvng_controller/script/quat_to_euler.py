#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'auvng_controller'
import roslib; roslib.load_manifest(PKG)
import rospy
import tf

# ROS messages.
from nav_msgs.msg import Odometry
from auvng_controller.msg import Eulers

class QuatToEuler():
    def __init__(self):
        self.got_new_msg = False
        self.euler_msg = Eulers()

        # Create subscribers and publishers.
        sub_odom  = rospy.Subscriber("auvng/state", Odometry, self.odom_callback)
        pub_euler = rospy.Publisher("euler", Eulers, queue_size=100)

        # Main while loop.
        while not rospy.is_shutdown():
            # Publish new data if we got a new message.
            if self.got_new_msg:
                pub_euler.publish(self.euler_msg)
                self.got_new_msg = False

    # Odometry callback function.
    def odom_callback(self, msg):
        # Convert quaternions to Euler angles.
        (r, p, y) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.fill_euler_msg(msg, r, p, y)

    # Fill in Euler angle message.
    def fill_euler_msg(self, msg, r, p, y):
        self.got_new_msg = True
        self.euler_msg.header.stamp = msg.header.stamp
        self.euler_msg.roll  = r
        self.euler_msg.pitch = p
        self.euler_msg.yaw   = y

# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('quat_to_euler')
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        quat_to_euler = QuatToEuler()
    except rospy.ROSInterruptException: pass