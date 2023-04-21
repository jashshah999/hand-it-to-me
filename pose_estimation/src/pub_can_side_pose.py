# #!/usr/bin/env python
# import rospy
# from cv_bridge import CvBridge, CvBridgeError
# from sensor_msgs.msg import Image
# from geometry_msgs.msg import PoseStamped
# import cv2
# import numpy as np

# class KinectPoseEstimator:
#     def __init__(self):
#         self.bridge = CvBridge()
#         self.image_sub = rospy.Subscriber("/kinect/image", Image, self.image_callback)
#         self.pose_pub = rospy.Publisher("/rectangle_pose", PoseStamped, queue_size=10)
#         self.image_width = 640 
#         self.image_height = 480 

#     def image_callback(self, data):
#         try:
#             cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
#         except CvBridgeError as e:
#             rospy.logerr(e)
#         gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
#         ret, thresh = cv2.threshold(gray_image, 127, 255, 0)
#         contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
#         max_area = 0
#         best_cnt = None
#         for cnt in contours:
#             area = cv2.contourArea(cnt)
#             if area > max_area and len(cnt) == 4:
#                 peri = cv2.arcLength(cnt,True)
#                 approx = cv2.approxPolyDP(cnt,0.1*peri,True)
#                 if len(approx) == 4:
#                     max_area = area
#                     best_cnt = cnt
#         if best_cnt is not None:
#             depth_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
#             x,y,w,h = cv2.boundingRect(best_cnt)
#             top_left_depth = depth_image[y, x]
#             top_right_depth = depth_image[y, x+w]
#             bottom_left_depth = depth_image[y+h, x]
#             bottom_right_depth = depth_image[y+h, x+w]

#             avg_depth = np.mean([top_left_depth, top_right_depth, bottom_left_depth, bottom_right_depth])
#             cx = x + w/2
#             cy = y + h/2
#             fx = fy = 525.0 
#             cx = self.image_width / 2.0
#             cy = self.image_height / 2.0
#             x_kinect = (cx - cx) * avg_depth / fx
#             y_kinect = (cy - cy) * avg_depth / fy
#             z_kinect = avg_depth
#             rectangle_pose = PoseStamped()
#             rectangle_pose.header.stamp = rospy.Time.now()
#             rectangle_pose.header.frame_id = "/kinect_frame"
#             rectangle_pose.pose
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from geometry_msgs.msg import Pose2D
import numpy as np

class QuadrilateralDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/rgb/image_raw", Image, self.image_callback)
        self.pose_pub = rospy.Publisher("/quadrilateral/pose", Pose2D, queue_size=1)
        self.focal_length = 500.0 # focal length of the camera in pixels
        self.square_size = 1.0 # size of the quadrilateral in meters

    def image_callback(self, data):
        print("I am in callback")
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Detect edges using Canny edge detection
        edges = cv2.Canny(gray_image, 50, 150)

        # Find contours in the image
        contours, hierarchy = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Find quadrilaterals among the contours
        quadrilaterals = []

  
        for cnt in contours:
            approx = cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True)
            if len(approx) == 4 and cv2.isContourConvex(approx):
                quadrilaterals.append(approx.reshape(-1, 2))

        # Choose the largest quadrilateral as the target
        if len(quadrilaterals) > 0:
            target = max(quadrilaterals, key=cv2.contourArea)
            # Draw the quadrilateral on the image
            cv2.drawContours(cv_image, [target.reshape(-1, 1, 2)], -1, (0, 255, 0), 3)

            # Compute the 2D pose of the quadrilateral
            object_points = np.array([(0, 0, 0),
                                      (self.square_size, 0, 0),
                                      (self.square_size, self.square_size, 0),
                                      (0, self.square_size, 0)], dtype=np.float32)
            image_points = np.array([target[0], target[1], target[2], target[3]], dtype=np.float32)
            camera_matrix = np.array([[self.focal_length, 0, cv_image.shape[1] / 2],
                                      [0, self.focal_length, cv_image.shape[0] / 2],
                                      [0, 0, 1]], dtype=np.float32)
            dist_coeffs = np.zeros((4, 1), dtype=np.float32)
            _, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs)

            # Convert the rotation vector to Euler angles
            theta = np.arctan2(rvec[1, 0], rvec[0, 0])
            pose = Pose2D(x=tvec[0, 0], y=tvec[1, 0], theta=theta)
            self.pose_pub.publish(pose)

        # Display the image
        cv2.imshow("Image", cv_image)
        cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('quadrilateral_detector')
    detector = QuadrilateralDetector()
    rospy.spin()