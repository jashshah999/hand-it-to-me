

from cmath import sqrt
import rospy
from geometry_msgs.msg import PoseStamped
from frankapy import FrankaArm
from std_msgs.msg import Bool
import numpy as np

fa = FrankaArm()
flag1 = 1
flag2 = 1
flag3 = 1
flag4 = 1
flag5 = 1
flag6 = 1
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
    global flag4
    global flag5
    global flag6
    global fa
    global goal
    print("Can detected")
    if flag1 == 1:
        fa.open_gripper()
        goal = data
        goal.pose.position.x += 0.17
        flag1 = 0
        flag2 = 1
        flag3 = 1
        flag4 = 1
        flag5 = 1
        flag6 = 1
        print("Reaching can at: ", goal)
    
    goal.header.frame_id = ""
    force_torque = fa.get_ee_force_torque()
    tot_force = np.sqrt((force_torque[0]**2+force_torque[1]**2+force_torque[2]**2))
    # print("Force on gripper = ", tot_force)
    if force_torque[2] > 20:
        print("Opening Gripper")
        fa.open_gripper()
        flag3 = 0

    if reached_goal(goal):
        if flag2 == 1:
            print("Temp")
            # goal.pose.position.x=goal.pose.position.x
            # goal.pose.position.y=goal.pose.position.y
            # goal.pose.position.z=0.13
            if flag5 == 1:
                goal.pose.position.z -= 0.04
                flag5 = 0
            # goal.pose.orientation.x = 1
            # goal.pose.orientation.y = 0
            # goal.pose.orientation.z = 0
            # goal.pose.orientation.w = 0
        if reached_goal(goal):
            flag2 = 0
            if flag3 == 1:
                print("Closing Gripper")
                fa.close_gripper()
                if fa.get_gripper_width() < 0.05:
                    print("Can not Picked")
                    reset_pub.publish(True)
                    flag1 = 1
                    return
                # if force_torque[2] < 5:
                #     print("Can not Picked.")
                #     pass
                # width = fa.get_gripper_width
                # fa.goto_gripper(0.05)
                flag3 = 0

            # Update the goal to home position
            # print("Reaching hand at = ", hand_pose)
            goal.pose.position.x = 0.49517871 #0.247916467066 #0.38900448 #0.990427905532281
            goal.pose.position.y = 0.2813782 #-0.215159624592 #0.25938477 #0.34089906849512275
            goal.pose.position.z = 4.86925653e-01 #0.9
            # goal.pose.position.x = 0.990427905532281
            # goal.pose.position.y = 0.34089906849512275
            # goal.pose.position.z = 0.9
            goal.pose.orientation.x = 1
            goal.pose.orientation.y = 0
            goal.pose.orientation.z = 0
            goal.pose.orientation.w = 0
            # if flag6 == 1:
            #     goal.pose.position.x = hand_pose.pose.position.x - 0.3 # 0.38900448
            #     flag6 = 0
            # goal.pose.position.y = hand_pose.pose.position.y # 0.25938477
            # goal.pose.position.z = hand_pose.pose.position.z # 0.39969875
            # goal.pose.orientation.x = hand_pose.pose.orientation.x
            # goal.pose.orientation.y = hand_pose.pose.orientation.y
            # goal.pose.orientation.z = hand_pose.pose.orientation.z
            # goal.pose.orientation.w = hand_pose.pose.orientation.w
            # if force_torque[2] < 0.5:
            # if tot_force < 0.5:
            if fa.get_gripper_width() < 0.05:
                print("Can fell")
                reset_pub.publish(True)
                flag1 = 1
                return
            # if reached_goal(goal):
            #     flag4 = 0
                # print("Flag 1 = ", 1)
                # flag1 = 1
            # goal.pose.position.z = 0.34548961 #0.286925653 #4.86925653e-01
            # goal.pose.orientation.x = hand_pose.pose.orientation.x
            # goal.pose.orientation.y = hand_pose.pose.orientation.y
            # goal.pose.orientation.z = hand_pose.pose.orientation.z
            # goal.pose.orientation.w = hand_pose.pose.orientation.w
    # if (flag4 == 1):
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
    # current_pose = [x,y,z]
    # print()
    if abs(x - goal.pose.position.x) < 0.01 and abs(y - goal.pose.position.y) < 0.01 and abs(z - goal.pose.position.z) < 0.01:
        print("Reached the goal")
        return True
    else:
        return False


if __name__ == '__main__':
    # rospy.init_node('pose_subscriber_publisher', anonymous=True)
    sub = rospy.Subscriber('/detected_circle_pose', PoseStamped, callback)
    pub = rospy.Publisher('/goal_location', PoseStamped, queue_size=10)
    reset_pub = rospy.Publisher('/reset_required', Bool, queue_size=10)
    hand_sub = rospy.Subscriber('/output_pose', PoseStamped, get_handpose_callback)
    rospy.spin()
