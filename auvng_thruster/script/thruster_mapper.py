#!/usr/bin/env python
import rospy
from numpy  import *
from geometry_msgs.msg import TwistStamped
from auvng_thruster.msg import thruster_torque

class thrust_mapper:
    def __init__(self):
        rospy.init_node('thrust_mapper')
        self.thruster_pub = rospy.Publisher('auvng/thruster_torque', thruster_torque, queue_size=10)
        rospy.Subscriber("auvng/control_effort", TwistStamped, self.thruster_map_callback)
        l_y = 0.12
        l_x = 0.15
        l_15 = 0.20
        l_37 = 0.25
        self.P = array([[0,     0,       1,       0,       0,       0,       1,       0],
			[-1,     0,       0,       0,       1,       0,       0,       0],
			[0,     -1,       0,      -1,       0,      -1,       0,      -1],
			[0,    l_x,       0,     l_x,       0,    -l_x,       0,    -l_x],
            [0,   -l_y,       0,     l_y,       0,     l_y,       0,    -l_y],
			[-l_15,  0,   -l_37,       0,   -l_15,       0,    l_37,       0]])

        print(self.P)

        self.M = linalg.pinv(self.P)

        print(self.M)

    
    def thruster_map_callback(self , message):
        F = array([message.twist.linear.x, message.twist.linear.y, message.twist.linear.z,
		   	message.twist.angular.x, message.twist.angular.y, message.twist.angular.z])
        
        t_torque = dot(self.M, F.T)

        print(t_torque)

        thruster = thruster_torque()

        for i in range(8):
            thruster_torque.data[i] = t_torque[i]
        

if __name__ == '__main__':
     x = thrust_mapper()
    #x.publishOdom()
    # rospy.spin()