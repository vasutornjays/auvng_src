import rospy
import math
from geometry_msgs.msg import TwistStamped

rospy.init_node('mission_planner', anonymous=True)

current_time = rospy.Time.now()
last_time = rospy.Time.now()

def transform_callback(data):

    current_time = rospy.Time.now()
    pub = rospy.Publisher('set_pose', TwistStamped, queue_size= 100)
    
    pose = PoseStamped()
    data.header.stamp = current_time

    r_x = math.radians(data.twist.angular.x)
    r_y = math.radians(data.twist.angular.y)
    r_z = math.radians(data.twist.angular.z)

    pose.pose.position = translation_pose.point
    pose.pose.orientation.x = pose_quat[0]
    pose.pose.orientation.y = pose_quat[1]
    pose.pose.orientation.z = pose_quat[2]
    pose.pose.orientation.w = pose_quat[3]


    pub.publish(pose)

    last_time = current_time

def mission_planner():
    
    rospy.Subscriber("set_point", geometry_msgs::TwistStamped , transform_callback)

    rospy.spin()

    

if __name__ ==  '__main__':
    try:
        mission_planner()
    except rospy.ROSInternalException:
        pass