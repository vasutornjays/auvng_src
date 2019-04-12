#!/usr/bin/env python
import rospy, math, tf
from geometry_msgs.msg import TwistStamped, Twist, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Bool

class auvMissionCommand:
    def __init__(self):
        rospy.init_node('auv_mission_node')
        self.command = TwistStamped()
        self.pose = Pose()

        self.current_state = [0, 0, 0, 0, 0, 0]

        self.pub_command = rospy.Publisher('/auvng/set_point', TwistStamped, queue_size=100)
        
        rospy.Subscriber('user_set_point', Twist, self.userSetPoint)
        # rospy.Subscriber('/auvng/state', Odometry, self.getCurrentState)


    def getCurrentState(self, data):
        self.pose = data.pose.pose
        pose = self.pose

        temp = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        euler_angular = tf.transformations.euler_from_quaternion(temp)


        self.current_state[0] = pose.position.x
        self.current_state[1] = pose.position.y
        self.current_state[2] = pose.position.z

        self.current_state[3] = euler_angular[0]
        self.current_state[4] = euler_angular[1]
        self.current_state[5] = euler_angular[2]

    def userSetPoint(self, data):
        self.command.header.stamp = rospy.Time.now()
        self.command.twist = data
        self.pub_command.publish(self.command)

if __name__=='__main__':
    auvng = auvMissionCommand()
    while not rospy.is_shutdown():
        rospy.sleep(0.5)
        print("running node")
                
    # count = 5
    # while not rospy.is_shutdown() and not zeabus_robot.isFail(count):
    #     zeabus_robot.move('',1)
    #     rospy.loginfo('\nx: %f\ny: %f\nz: %f'%(zeabus_robot.auv_state[0], zeabus_robot.auv_state[1], zeabus_robot.auv_state[2]))
    #     count-=1

