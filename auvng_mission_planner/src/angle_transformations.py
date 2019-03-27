import rospy
import tf
import message_filters
import math
from geometry_msgs.msg import PoseStamped, Vector3Stamped, PointStamped

rospy.init_node('mission_planner', anonymous=True)

current_time = rospy.Time.now()
last_time = rospy.Time.now()

def transform_callback(angle_pose, translation_pose):

    current_time = rospy.Time.now()
    pub = rospy.Publisher('set_pose', PoseStamped, queue_size= 100)
    
    pose = PoseStamped()
    pose.header.stamp = current_time

    r_x = math.radians(angle_pose.vector.x)
    r_y = math.radians(angle_pose.vector.y)
    r_z = math.radians(angle_pose.vector.z)
    pose_quat = tf.transformations.quaternion_from_euler(r_x, r_y, r_z, 'ryxz')

    pose.pose.position = translation_pose.point
    pose.pose.orientation.x = pose_quat[0]
    pose.pose.orientation.y = pose_quat[1]
    pose.pose.orientation.z = pose_quat[2]
    pose.pose.orientation.w = pose_quat[3]


    pub.publish(pose)

    last_time = current_time

def mission_planner():
    
    angle_sub = message_filters.Subscriber('angle_pose', Vector3Stamped)
    pose_sub = message_filters.Subscriber('translation_pose', PointStamped)

    ts = message_filters.TimeSynchronizer([angle_sub, pose_sub], 10)
    ts.registerCallback(transform_callback)
    rospy.spin()

    

if __name__ ==  '__main__':
    try:
        mission_planner()
    except rospy.ROSInternalException:
        pass