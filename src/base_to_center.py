#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

rospy.init_node('base_to_center')
listener = tf.TransformListener()
pub = rospy.Publisher("/fuse_pose_center", PoseWithCovarianceStamped, queue_size=50)

def callback(msg):
    p = PoseStamped()
    p.header.frame_id = "center_link"
    p.pose.orientation.w = 1.0
    listener.waitForTransform("/map", "/center_link", rospy.Time(0), rospy.Duration(3))
    # Obtain the center pose with respect to /map
    p = listener.transformPose("/map", p)

    p2 = PoseWithCovarianceStamped()
    p2.pose.pose = p.pose
    p2.pose.covariance = msg.pose.covariance
    # Use the original message's header so the stamp remains same
    p2.header = msg.header
    pub.publish(p2)

    # print("base  : x = %2.3f, y = %2.3f"%(msg.pose.pose.position.x, msg.pose.pose.position.y))
    # print("center: x = %2.3f, y = %2.3f"%(p2.pose.pose.position.x, p2.pose.pose.position.y))

fuse_sub = rospy.Subscriber("/odometry/filtered", Odometry, callback)
rospy.spin()
