#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from frankapy import FrankaArm

fa = FrankaArm()

def callback(data):
    # change the frame_id and keep everything else the same
    data.header.frame_id = ""
    # change the x position of the goal pose by 0.1
    data.pose.position.x += 0.17
    # Check if the robot has reached the goal pose, if yes then move the robot to z =0.13 for the same x and y position
    if fa.reached_goal(data):
        data.pose.position.z = 0.13
    pub.publish(data)

if __name__ == '__main__':
    rospy.init_node('pose_subscriber_publisher', anonymous=True)
    sub = rospy.Subscriber('/detected_circle_pose', PoseStamped, callback)
    pub = rospy.Publisher('/goal_location', PoseStamped, queue_size=10)
    rospy.spin()
