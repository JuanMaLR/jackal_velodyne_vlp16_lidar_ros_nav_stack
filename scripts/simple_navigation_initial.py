#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

rospy.init_node('pub_initialpose_node', anonymous=True)
pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
initialpose_msg = PoseWithCovarianceStamped()
#For simulation
initialpose_msg.header.frame_id = "map"
initialpose_msg.pose.pose.position.x = -0.4696083068847656
initialpose_msg.pose.pose.position.y = 0.12282633781433105
initialpose_msg.pose.pose.orientation.z = -0.13993019689107583
initialpose_msg.pose.pose.orientation.w = 0.9901613706856195
rate = rospy.Rate(1)

while not rospy.is_shutdown():
    pub.publish(initialpose_msg)
    rate.sleep()