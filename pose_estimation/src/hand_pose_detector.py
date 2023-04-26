
#!/usr/bin/env python

import cv2
import numpy as np
import mediapipe as mp
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped 
import rospy
import pyrealsense2 as rs2

class HandPoseDetector:
    def __init__(self, rgb_topic, depth_topic, depth_output, annotated_image, hand_pose):
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(static_image_mode=False, max_num_hands=2, min_detection_confidence=0.9)
        self.mp_drawing = mp.solutions.drawing_utils
        rospy.init_node('hand_pose_detection')
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(rgb_topic, Image, self.image_callback)
        self.pub = rospy.Publisher(annotated_image, Image, queue_size=10)
        self.pose_pub = rospy.Publisher(hand_pose,PoseStamped, queue_size =10)
        self.depth_sub = rospy.Subscriber(depth_topic, Image, self.depth_callback)
        self.depth_pub = rospy.Publisher(depth_output, PoseStamped, queue_size=10)
        self.camera_mat = np.array([[615.8245849609375, 0.0, 322.6065368652344, 0.0, 0.0, 616.0632934570312, 235.29945373535156, 0.0, 0.0, 0.0, 1.0, 0.0]])
        self.intrinsic = np.array([[615.8245849609375, 0.0, 322.6065368652344], 
                          [0.0, 616.0632934570312, 235.29945373535156], 
                          [0.0, 0.0, 1.0]])
        self.fx = self.intrinsic[0, 0]    
        self.fy = self.intrinsic[1, 1]
        self.cx = self.intrinsic[0, 2]
        self.cy = self.intrinsic[1, 2]
        self.rotation = np.array([[0.998942 ,-0.007404 ,-0.045164],       
                        [0.045203 ,0.005115 ,0.998955],
                        [-0.007166 ,-0.999960 ,0.005445]])
        self.translation = np.array([[0.657895 ,-0.567591 ,0.528885 ]]).reshape(3, 1)
        self.transform = np.hstack((self.rotation, self.translation))
        self.x_rgb = 0
        self.y_rgb = 0
        self.z_depth = 0
        self.cx_cam_frame = 0
        self.cy_came_frame = 0
        self.color_intrinsics = rs2.pyrealsense2.intrinsics()
        self.color_intrinsics.fx = self.fx
        self.color_intrinsics.fy = self.fy
        self.color_intrinsics.ppx = self.cx
        self.color_intrinsics.ppy = self.cy
        self.color_intrinsics.height = 480
        self.color_intrinsics.width = 848

    def depth_callback(self,msg):
        depth_array = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.z_depth = depth_array[self.y_rgb,self.x_rgb]/1000

    def image_callback(self, msg):
        color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        results = self.hands.process(color_image)
        hand_pose = []
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                self.mp_drawing.draw_landmarks(
                    color_image, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                for id,lm in enumerate(hand_landmarks.landmark):
                    h,w,_ = color_image.shape
                    cx_image,cy_image = int(lm.x*w),int(lm.y*h)
                    self.x_rgb = cx_image
                    self.y_rgb = cy_image
                    x,y,z = rs2.rs2_deproject_pixel_to_point(self.color_intrinsics,[self.x_rgb,self.y_rgb],self.z_depth)
                    camera_coordinates = np.array([[x,y,z,1]]).T
                    base_coordinates = np.matmul(self.transform,camera_coordinates)
                    if(id==9) :
                        hand_pose = PoseStamped()
                        hand_pose.pose.position.x = base_coordinates[0]
                        hand_pose.pose.position.y =base_coordinates[1]
                        hand_pose.pose.position.z = base_coordinates[2]
                        hand_pose.pose.orientation.x = 1
                        hand_pose.pose.orientation.y = 0
                        hand_pose.pose.orientation.z=0
                        hand_pose.pose.orientation.w=0
        annotated_image_msg = self.bridge.cv2_to_imgmsg(color_image, encoding='rgb8')
        self.pub.publish(annotated_image_msg)
        self.pose_pub.publish(hand_pose)

    def run(self):
        rospy.spin()
        self.hands.close()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    detector = HandPoseDetector('/camera/color/image_raw','camera/aligned_depth_to_color/image_raw','/depth_output', '/annotated_image','/hand_pose')
    detector.run()
