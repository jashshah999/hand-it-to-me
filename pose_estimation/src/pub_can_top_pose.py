#!/usr/bin/env python

import statistics
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from collections import deque
     

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
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, dp=1, minDist=self.min_distance,
                                param1=self.threshold, param2=30, minRadius=self.min_radius,
                                maxRadius=self.max_radius)
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            for (x, y, r) in circles:
                cv2.circle(frame, (x, y), r, (0, 255, 0), 2)
                self.queue_radius.append(r)
                self.queue_pose_x.append(x)
                self.queue_pose_y.append(y)
                self.pose_msg.pose.position.x = statistics.median(self.queue_pose_x)
                self.pose_msg.pose.position.y = statistics.median(self.queue_pose_y)
                radius = statistics.median(self.queue_radius)
                print(radius)
                self.pose_msg.pose.position.z = 0.0
                self.pose_msg.pose.orientation.x = 0.0
                self.pose_msg.pose.orientation.y = 0.0
                self.pose_msg.pose.orientation.z = 0.0
                self.pose_msg.pose.orientation.w = 1.0
                self.output_pub.publish(self.pose_msg)

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
