import mediapipe as mp
import cv2
import numpy as np
import statistics

def get_hand_pose(frame, vis_flag = 1):
    mp_drawing = mp.solutions.drawing_utils
    mp_hands = mp.solutions.hands
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

def get_can_pose_vertical_template(frame):
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


def get_can_pose_horizontal_hsv(frame):
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


def get_can_pose(frame, queue_x, queue_y, queue_radius,min_radius,max_radius,threshold,min_distance,pose_msg):
    camera_mat = np.array([[975.9609985351562, 0.0, 1018.8323974609375, 0.0, 0.0, 975.6666870117188, 777.054931640625, 0.0, 0.0, 0.0, 1.0, 0.0]])

    intrinsic = np.array([[975.9609985351562, 0.0, 1018.8323974609375], 
                          [0.0, 975.6666870117188, 777.054931640625], 
                          [0.0, 0.0, 1.0]])

    fx = intrinsic[0, 0]
    fy = intrinsic[1, 1]
    cx = intrinsic[0, 2]
    cy = intrinsic[1, 2]

    rotation = np.array([[-0.05437199,  0.99769028, -0.04071191],       
                            [0.99807252,  0.05551034,  0.02739719],
                            [0.02959404, -0.03914623, -0.99878567]])

    translation = np.array([[0.64138432 , 0.1008975 ,  0.831477 ]]).reshape(3, 1)

    transform = np.hstack((rotation, translation))

    z =0.685

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, dp=1, minDist=min_distance,
                                param1=threshold, param2=30, minRadius=min_radius,
                                maxRadius=max_radius)
    
    if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            for (x, y, r) in circles:
                cv2.circle(frame, (x, y), r, (0, 255, 0), 2)
                x_cam_frame = ((x-cx)/fx)*z
                y_cam_frame = ((y-cx)/fx)*z
                camera_cord = np.array([[x_cam_frame, y_cam_frame, z, 1]]).T
                base_cord = np.matmul(transform, camera_cord)
                print(base_cord)
                queue_radius.append(r)
                queue_x.append(base_cord[0])
                queue_y.append(base_cord[1])
                pose_msg.pose.position.x = statistics.median(queue_x)
                pose_msg.pose.position.y = statistics.median(queue_y)
                radius = statistics.median(queue_radius)
                pose_msg.pose.position.z = 0.17
                pose_msg.pose.orientation.x = 1
                pose_msg.pose.orientation.y = 0
                pose_msg.pose.orientation.z = 0
                pose_msg.pose.orientation.w = 0
                return pose_msg
    else:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([110, 50, 50])
        upper_blue = np.array([112, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            width = int(rect[1][0])
            height = int(rect[1][1])
            if width > 70 and height > 70:
                cv2.drawContours(frame, [box], 0, (0, 255, 0), 2)
                cv2.imshow("frame",frame)
                angle = rect[2]
                roll =np.deg2rad(angle) 
                pitch = 0
                yaw = 0
                min_x = np.min(box[:,0])
                min_y = np.min(box[:,1])
                max_x = np.max(box[:,0])
                max_y = np.max(box[:,1])
                min_x_cam_frame = ((min_x-cx)/fx)*z
                min_y_cam_frame = ((min_y-cx)/fx)*z
                max_x_cam_frame = ((max_x-cx)/fx)*z
                max_y_cam_frame = ((max_y-cx)/fx)*z
                camera_cord_min = np.array([[min_x_cam_frame, min_y_cam_frame, z, 1]]).T
                camera_cord_max = np.array([[max_x_cam_frame, max_y_cam_frame, z, 1]]).T 
                base_coord_min = np.matmul(transform, camera_cord_min)
                base_coord_max = np.matmul(transform, camera_cord_max)                
                pose_msg.pose.position.x = (base_coord_min[0]+ base_coord_max[0])/2 
                pose_msg.pose.position.y = (base_coord_min[1]+base_coord_max[1])/2
<<<<<<< HEAD:pose_estimation/src/pose_functions.py
                # pose_msg.pose.position.y = 1
                # pose_msg.pose.position.x = 0
                pose_msg.pose.position.z = 0.065
                # pose_msg.pose.orientation.x = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
                # pose_msg.pose.orientation.y =  np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
                # pose_msg.pose.orientation.z = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
                # pose_msg.pose.orientation.w = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
                pose_msg.pose.orientation.x = 1
                pose_msg.pose.orientation.y = 0
                pose_msg.pose.orientation.z = 0
                pose_msg.pose.orientation.w = 0
                # cv2.imshow("frame",frame)
            # cv2.imshow("image",frame)
=======
                pose_msg.pose.position.z = 0.025
                pose_msg.pose.orientation.x = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
                pose_msg.pose.orientation.y =  np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
                pose_msg.pose.orientation.z = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
                pose_msg.pose.orientation.w = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
>>>>>>> cdf0b24f965715ee36401cf5ea917ff3046f3626:pose_estimation/src/utils.py
                return pose_msg




