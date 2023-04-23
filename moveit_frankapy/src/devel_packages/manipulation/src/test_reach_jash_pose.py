

import rospy
from geometry_msgs.msg import PoseStamped
from frankapy import FrankaArm

fa = FrankaArm()
flag = 1
goal = PoseStamped()

# def callback(data):
#     global flag
#     global fa
#     global goal
#     if flag == 1:
#         goal = data
#         flag = 0
#     # change the frame_id and keep everything else the same
#     data.header.frame_id = ""
#     # change the x position of the goal pose by 0.1
#     data.pose.position.x += 0.17
#     goal.pose.position.x = data.pose.position.x
#     if reached_goal(goal):
#         data.pose.position.x=goal.pose.position.x
#         data.pose.position.y=goal.pose.position.y
#         data.pose.position.z=0.13
#         data.pose.orientation.x = 1
#         data.pose.orientation.y = 0
#         data.pose.orientation.z = 0
#         data.pose.orientation.w = 0
#         # flag = 1
#         # if reached_goal(data):
#         #     fa.close_gripper()
#         #     flag = 1
#     pub.publish(data)

def callback(data):
    global flag
    global fa
    global goal
    if flag == 1:
        goal = data
        goal.pose.position.x += 0.17
        # print("goal = ", goal)
        flag = 0
    # change the frame_id and keep everything else the same
    goal.header.frame_id = ""
    # change the x position of the goal pose by 0.1

    # goal.pose.position.x = data.pose.position.x
    if reached_goal(goal):
        goal.pose.position.x=goal.pose.position.x
        goal.pose.position.y=goal.pose.position.y
        goal.pose.position.z=0.13
        goal.pose.orientation.x = 1
        goal.pose.orientation.y = 0
        goal.pose.orientation.z = 0
        goal.pose.orientation.w = 0
        # flag = 1
        if reached_goal(goal):
            fa.close_gripper()
            
            # flag = 1
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
    rospy.spin()
