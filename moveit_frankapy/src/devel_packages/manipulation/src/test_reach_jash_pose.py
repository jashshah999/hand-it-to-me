#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped

def callback(data):
    # change the frame_id and keep everything else the same
    data.header.frame_id = ""
    # change the x position of the goal pose by 0.1
    data.pose.position.x += 0.17

    pub.publish(data)

if __name__ == '__main__':
    rospy.init_node('pose_subscriber_publisher', anonymous=True)
    sub = rospy.Subscriber('/detected_circle_pose', PoseStamped, callback)
    pub = rospy.Publisher('/goal_location', PoseStamped, queue_size=10)
    rospy.spin()
