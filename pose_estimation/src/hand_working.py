
#!/usr/bin/env python

import cv2
import numpy as np
import mediapipe as mp
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import rospy

class HandPoseDetector:
    def __init__(self, input_topic, output_topic):
        # Initialize Mediapipe hand detection module
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(static_image_mode=False, max_num_hands=2, min_detection_confidence=0.9)
        self.mp_drawing = mp.solutions.drawing_utils

        # Initialize ROS node
        rospy.init_node('hand_pose_detection')

        # Initialize OpenCV bridge
        self.bridge = CvBridge()

        # Subscribe to input image topic
        self.sub = rospy.Subscriber(input_topic, Image, self.image_callback)

        # Define publisher for annotated image topic
        self.pub = rospy.Publisher(output_topic, Image, queue_size=10)

    def image_callback(self, msg):
        # Convert ROS image to OpenCV image
        color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

        # Process image with Mediapipe
        results = self.hands.process(color_image)

        # If hands are detected, annotate the color image
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                self.mp_drawing.draw_landmarks(
                    color_image, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)

        # Convert OpenCV image back to ROS image
        annotated_image_msg = self.bridge.cv2_to_imgmsg(color_image, encoding='rgb8')

        # Publish annotated image to topic
        self.pub.publish(annotated_image_msg)

    def run(self):
        # Spin ROS node
        rospy.spin()

        # Clean up
        self.hands.close()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    detector = HandPoseDetector('/camera/color/image_raw', '/annotated_image')
    detector.run()
