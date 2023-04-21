import mediapipe as mp
import cv2
import numpy as np
import uuid
import os

mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands

def get_hand_pose(frame, vis_flag = 1):
    with mp_hands.Hands(min_detection_confidence=0.8, min_tracking_confidence=0.5) as hands:

        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image = cv2.flip(image, 1)
        image.flags.writeable = False
        results = hands.process(image)
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        hand_pose = []
        if results.multi_hand_landmarks:
            for handlms in results.multi_hand_landmarks:
                for id, lm in enumerate(handlms.landmark):
                    h,w,c = frame.shape
                    cx,cy = int(lm.x *w), int(lm.y*h)
                    if(id==9):
                        hand_pose.append(cx,cy)





        if(vis_flag):
            if results.multi_hand_landmarks:
                for num, hand in enumerate(results.multi_hand_landmarks):
                    mp_drawing.draw_landmarks(image, hand, mp_hands.HAND_CONNECTIONS, 
                                        mp_drawing.DrawingSpec(color=(121, 22, 76), thickness=2, circle_radius=4),
                                        mp_drawing.DrawingSpec(color=(250, 44, 250), thickness=2, circle_radius=2),
                                        )
            
    return hand_pose

def get_can_pose_vertical(frame):
    template_circle = cv2.imread('template_circle.png')
    w, h = template_circle.shape[::-1]
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    res_circle = cv2.matchTemplate(gray_frame, template_circle,cv2.TM_CCOEFF_NORMED)
    _,_,_, max_loc = cv2.minMaxLoc(res_circle)
    top_left = max_loc
    bottom_right = (top_left[0] + w, top_left[1]+ h)
    bottom_left = (bottom_right[0]-w,top_left[1])
    top_right = (top_left[0]+w, bottom_right[1]-h)
    
    return top_left, top_right, bottom_left, bottom_right


def get_can_pose_horizontal(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) 

    l_h = 100 # 128
    l_s = 100 # 100
    l_v = 20 # 20
    u_h = 133 # 133
    u_s = 255 # 255
    u_v = 255 # 255

    lower_thresh = np.array([l_h, l_s, l_v])
    upper_thresh = np.array([u_h, u_s, u_v])
    mask = cv2.inRange(hsv, lower_thresh, upper_thresh)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.erode(mask, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        approx = cv2.approxPolyDP(cnt, 0.02*cv2.arcLength(cnt, True), True)
        if len(approx) == 4:

            identifier_coordinates = np.squeeze(approx, axis=1)
            return identifier_coordinates





