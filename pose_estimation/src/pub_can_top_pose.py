#!/usr/bin/env python

import statistics
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from collections import deque
from pose_functions import *

class CircleDetector:

    def __init__(self):
        rospy.init_node('circle_detection', anonymous=True)
        self.bridge = CvBridge()
        self.min_radius = 30
        self.max_radius = 35
        self.threshold = 50
        self.min_distance = 30
        self.output_pub = rospy.Publisher('detected_circle_pose', PoseStamped, queue_size=1)
        self.pose_msg = PoseStamped()
        self.pose_msg.header.frame_id = "kinect"
        self.image_pub = rospy.Publisher('detected_circle_image', Image, queue_size=1)
        rospy.Subscriber('/rgb/image_raw', Image, self.image_callback)

        self.queue_pose_x = deque([],maxlen=10)
        self.queue_pose_y = deque([],maxlen=10)
        self.queue_radius = deque([], maxlen=10)

    def image_callback(self, msg):
    
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        self.can_top_pose = get_can_pose_vertical_hough(frame,self.queue_pose_x, self.queue_pose_y,self.queue_radius,self.min_radius,self.max_radius,self.threshold,self.min_distance,self.pose_msg)

        self.output_pub.publish(self.can_top_pose)

        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')     
        if image_msg is not None and len(image_msg.data) > 0:
            annotated_image = np.frombuffer(image_msg.data, np.uint8).reshape(image_msg.height, image_msg.width, -1)
            cv2.imshow('frame', annotated_image)
            cv2.waitKey(1)
            self.image_pub.publish(image_msg)
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    detector = CircleDetector()
    detector.run()
