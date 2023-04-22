# #!/usr/bin/env python
# import rospy
# import cv2
# import numpy as np
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge

# class FallenRectDetector:
#     def __init__(self):
#         rospy.init_node('fallen_rect_detector', anonymous=True)
#         self.bridge = CvBridge()
#         self.image_sub = rospy.Subscriber("/rgb/image_raw", Image, self.callback)

#     def callback(self, data):
#         img = self.bridge.imgmsg_to_cv2(data, "bgr8")

#         # Convert image to HSV color space
#         hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

#         # Define the range of the dark blue color in HSV
#         lower_blue = np.array([90, 50, 50])
#         upper_blue = np.array([130, 255, 255])

#         # Threshold the image to get only the dark blue pixels
#         mask = cv2.inRange(hsv, lower_blue, upper_blue)

#         # Find contours of the dark blue rectangles in the image
#         contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

#         # Loop through all the detected contours
#         for cnt in contours:
#             # Get the rectangle properties
#             x, y, w, h = cv2.boundingRect(cnt)

#             # Check if the rectangle is big enough to be a fallen rectangle
#             if w > 10 and h > 10:
#                 # Draw a green rectangle around the detected object
#                 cv2.rectangle(img, (x,y), (x+w,y+h), (0, 255, 0), 2)

#         # Display the resulting image
#         cv2.imshow("Image", img)
#         cv2.waitKey(1)

#     def run(self):
#         rospy.spin()

# if __name__ == '__main__':
#     detector = FallenRectDetector()
#     detector.run()

#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class FallenRectDetector:
    def __init__(self):
        rospy.init_node('fallen_rect_detector', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/rgb/image_raw", Image, self.callback)

    def callback(self, data):
        img = self.bridge.imgmsg_to_cv2(data, "bgr8")

        # Convert image to HSV color space
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Define the range of the dark blue color in HSV
        lower_blue = np.array([90, 50, 50])
        upper_blue = np.array([130, 255, 255])

        # Threshold the image to get only the dark blue pixels
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # Find contours of the dark blue rectangles in the image
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Loop through all the detected contours
        for cnt in contours:
            # Get the minimum area rectangle of the contour
            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect)
            box = np.int0(box)

            # Get the width and height of the rectangle
            width = int(rect[1][0])
            height = int(rect[1][1])

            # Check if the rectangle is big enough to be a fallen rectangle
            if width > 70 and height > 70:
                # Draw a green rectangle around the detected object
                cv2.drawContours(img, [box], 0, (0, 255, 0), 2)

                # Get the angle of the rectangle
                angle = rect[2]

                # Display the angle on the CV frame
                cv2.putText(img, f"Angle: {angle:.2f}", (box[1][0], box[1][1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # Display the resulting image
        cv2.imshow("Image", img)
        cv2.waitKey(1)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    detector = FallenRectDetector()
    detector.run()
