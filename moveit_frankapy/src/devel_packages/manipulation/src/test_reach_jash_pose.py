

import rospy
from geometry_msgs.msg import PoseStamped
from frankapy import FrankaArm

fa = FrankaArm()
flag1 = 1
flag2 = 1
flag3 = 1
hand_flag = 1
goal = PoseStamped()
fa.open_gripper()
hand_pose = PoseStamped()


def get_handpose_callback(data):
    global hand_pose
    # if hand_flag == 1:
        # change the frame_id and keep everything else the same
    hand_pose = data 
    hand_pose.header.frame_id = ""
    # change the x position of the goal pose by 0.1
    hand_pose.pose.position.x += 0.17
    hand_pose.pose.orientation.x = 1
    hand_pose.pose.orientation.y = 0
    hand_pose.pose.orientation.z = 0
    hand_pose.pose.orientation.w = 0
    # print("outer hand pose = ", hand_pose)

#     # hand_flag = 0

# def get_handpose_callback(data):
#     global goal
#     goal = data 
#     goal.header.frame_id = ""
#     # change the x position of the goal pose by 0.1
#     goal.pose.position.x += 0.17
#     goal.pose.orientation.x = 1
#     goal.pose.orientation.y = 0
#     goal.pose.orientation.z = 0
#     goal.pose.orientation.w = 0
#     print("outer hand pose = ", goal)
#     force_torque = fa.get_ee_force_torque()
#     if force_torque[2] > 6:
#         fa.open_gripper()
#     pub.publish(goal)
    

def callback(data):
    global flag1
    global flag2
    global flag3
    global fa
    global goal

    if flag1 == 1:
        goal = data
        goal.pose.position.x += 0.17
        flag1 = 0
    goal.header.frame_id = ""

    force_torque = fa.get_ee_force_torque()
    # print("Force on gripper = ", force_torque[2])
    if force_torque[2] > 6:
        print("Opening Gripper")
        fa.open_gripper()
        flag3 = 0

    if reached_goal(goal):
        if flag2 == 1:
            # goal.pose.position.x=goal.pose.position.x
            # goal.pose.position.y=goal.pose.position.y
            goal.pose.position.z=0.13
            # goal.pose.orientation.x = 1
            # goal.pose.orientation.y = 0
            # goal.pose.orientation.z = 0
            # goal.pose.orientation.w = 0
        if reached_goal(goal):
            flag2 = 0
            print("im in")
            if flag3 == 1:
                fa.close_gripper()
                flag3 = 0

            # Update the goal to home position
            print("inner ka inner hand pose = ", hand_pose)
            # goal.pose.position.x = 0.49517871 #0.247916467066 #0.38900448 #0.990427905532281
            # goal.pose.position.y = 0.2813782 #-0.215159624592 #0.25938477 #0.34089906849512275
            # goal.pose.position.z = 4.86925653e-01 #0.9
            # goal.pose.position.x = 0.990427905532281
            # goal.pose.position.y = 0.34089906849512275
            # goal.pose.position.z = 0.9
            goal.pose.orientation.x = 1
            goal.pose.orientation.y = 0
            goal.pose.orientation.z = 0
            goal.pose.orientation.w = 0
            goal.pose.position.x = hand_pose.pose.position.x # 0.38900448
            goal.pose.position.y = hand_pose.pose.position.y # 0.25938477
            # goal.pose.position.z = hand_pose.pose.position.z # 0.39969875
            goal.pose.position.z = 0.34548961 #0.286925653 #4.86925653e-01
            # goal.pose.orientation.x = hand_pose.pose.orientation.x
            # goal.pose.orientation.y = hand_pose.pose.orientation.y
            # goal.pose.orientation.z = hand_pose.pose.orientation.z
            # goal.pose.orientation.w = hand_pose.pose.orientation.w
    pub.publish(goal)

def reached_goal(goal):
    # Check if the current pose of the robot is close to the goal pose
    # If yes then return True else return False
    # print("here")
    curr_pose = fa.get_pose()
    x = curr_pose.translation[0]
    y = curr_pose.translation[1]
    z = curr_pose.translation[2]
    # print("here")
    current_pose = [x,y,z]
    # print()
    if abs(current_pose[0] - goal.pose.position.x) < 0.01 and abs(current_pose[1] - goal.pose.position.y) < 0.01 and abs(current_pose[2] - goal.pose.position.z) < 0.01:
        print("Reached the goal")
        return True
    else:
        return False


if __name__ == '__main__':
    # rospy.init_node('pose_subscriber_publisher', anonymous=True)
    sub = rospy.Subscriber('/detected_circle_pose', PoseStamped, callback)
    pub = rospy.Publisher('/goal_location', PoseStamped, queue_size=10)
    hand_sub = rospy.Subscriber('/output_pose', PoseStamped, get_handpose_callback)
    rospy.spin()
