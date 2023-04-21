#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge 
import cv2
import mediapipe as mp
from utils import get_hand_pose
import pyrealsense2
from realsense_depth import *



class TempPublisher:

    def __init__(self):
        # rospy.init_node('pub_hand_pose_node',anonymous=True)
        self.pub = rospy.Publisher('/pose_data', Image, queue_size=10)
        self.sub = rospy.Subscriber('/rgb/image_raw',Image, self.callback)
        self.br = CvBridge()

    def callback(self, data):
        # rospy.loginfo("receiving video frame")
        current_frame = self.br.imgmsg_to_cv2(data)
        pose_frame = get_hand_pose(current_frame)
        # rospy.loginfo('publishing frame')
        self.pub.publish(self.br.cv2_to_imgmsg(pose_frame,"rgb8"))

def main():
    rospy.init_node('pub_hand_pose_node',anonymous=True)
    TempPublisher()
    rate = rospy.Rate(10)
    rospy.spin()
    
if __name__ == '__main__':
    try: 
        main()
    except rospy.ROSInterruptException:
        pass



