
#!/usr/bin/env python

import cv2
import numpy as np
import mediapipe as mp
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped 
import rospy
import math 

camera_mat = np.array([[615.8245849609375, 0.0, 322.6065368652344, 0.0, 0.0, 616.0632934570312, 235.29945373535156, 0.0, 0.0, 0.0, 1.0, 0.0]])
# camera_mat = np.array([[616.8307495117188, 0.0, 310.5340881347656, 0.0, 0.0, 616.9888916015625, 236.6097869873047, 0.0, 0.0, 0.0, 1.0, 0.0]])
intrinsic = np.array([[615.8245849609375, 0.0, 322.6065368652344], 
                          [0.0, 616.0632934570312, 235.29945373535156], 
                          [0.0, 0.0, 1.0]])
# intrinsic = np.array([[616.8307495117188, 0.0, 310.5340881347656], 
                        #   [0.0, 616.9888916015625, 236.6097869873047], 
                        #   [0.0, 0.0, 1.0]])


fx = intrinsic[0, 0]
fy = intrinsic[1, 1]
cx = intrinsic[0, 2]
cy = intrinsic[1, 2]

rotation = np.array([[0.998942 ,-0.007404 ,-0.045164],       
                        [0.045203 ,0.005115 ,0.998955],
                        [-0.007166 ,-0.999960 ,0.005445]])

translation = np.array([[0.657895 ,-0.567591 ,0.528885 ]]).reshape(3, 1)

transform = np.hstack((rotation, translation))

z =0.9

class HandPoseDetector:
    def __init__(self, input_topic, output_image, output_pose):
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
        self.pub = rospy.Publisher(output_image, Image, queue_size=10)

        self.pose_pub = rospy.Publisher(output_pose,PoseStamped, queue_size =10)

    def image_callback(self, msg):
        # Convert ROS image to OpenCV image
        color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

        # Process image with Mediapipe
        results = self.hands.process(color_image)

        # If hands are detected, annotate the color image
        hand_pose = []
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                landmarks = results.multi_hand_landmarks[0]
                palm_center = landmarks.landmark[0]
                wrist = landmarks.landmark[self.mp_hands.HandLandmark.WRIST]
                palm_vector = [palm_center.x - wrist.x, palm_center.y - wrist.y, palm_center.z - wrist.z]
                x_axis = [1, 0, 0]
                palm_angle = cv2.fastAtan2(cv2.norm(np.cross(palm_vector, x_axis)), np.dot(palm_vector, x_axis))
                print(palm_angle)
                self.mp_drawing.draw_landmarks(
                    color_image, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                for id,lm in enumerate(hand_landmarks.landmark):
                    h,w,c = color_image.shape
                    cx_image,cy_image = int(lm.x*w),int(lm.y*h)

                    cx_cam_frame = ((cx_image - cx)/fx)*z
                    cy_cam_frame = ((cy_image - cy)/fy)*z

                    camera_coordinates = np.array([[cx_cam_frame,cy_cam_frame,z,1]]).T
                    base_coordinates = np.matmul(transform,camera_coordinates)
                    if(id==9) :
                        hand_pose = PoseStamped()
                        hand_pose.pose.position.x = base_coordinates[0]
                        hand_pose.pose.position.y =base_coordinates[1]
                        hand_pose.pose.position.z = 0.9
                        hand_pose.pose.orientation.x = 1
                        hand_pose.pose.orientation.y = 0
                        hand_pose.pose.orientation.z=0
                        hand_pose.pose.orientation.w=0

                        


        # Convert OpenCV image back to ROS image
        annotated_image_msg = self.bridge.cv2_to_imgmsg(color_image, encoding='rgb8')

        # Publish annotated image to topic
        self.pub.publish(annotated_image_msg)
        self.pose_pub.publish(hand_pose)

    def run(self):
        # Spin ROS node
        rospy.spin()

        # Clean up
        self.hands.close()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    detector = HandPoseDetector('/camera/color/image_raw', '/annotated_image','/output_pose')
    detector.run()
