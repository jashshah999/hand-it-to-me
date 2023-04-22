# Write a node that publishes a goal pose to the /goal_location topic as a geometry_msgs/PoseStamped message
import rospy
from geometry_msgs.msg import PoseStamped

def goal_pub():
    pub = rospy.Publisher('/goal_location', PoseStamped, queue_size=10)
    rospy.init_node('goal_pub', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    # Read the goal poses from an array and then publish them one by one once the robot reaches the previous goal
    goal = PoseStamped()
    goal.header.seq = 0
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = ""
    goal.pose.position.x = 0.4
    goal.pose.position.y = 0.0
    goal.pose.position.z = 0.4
    goal.pose.orientation.x = 0.0
    goal.pose.orientation.y = 0.707
    goal.pose.orientation.z = 0.0
    goal.pose.orientation.w = 0.707
    
    while not rospy.is_shutdown():
        # Keep incerementing the x position of the goal pose by 0.1
        goal = PoseStamped()
        goal.header.seq = 0
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = ""
        goal.pose.position.x = 0.4
        goal.pose.position.x += 0.1
        goal.pose.position.y = 0.0
        goal.pose.position.z = 0.4
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.707
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 0.707
        # Keep doing this until the x position of the goal pose reaches 0.8 then stop the node.
        if goal.pose.position.x >= 0.8:
            break
        rospy.loginfo(goal)
        pub.publish(goal)
        rate.sleep()

if __name__ == '__main__':
    try:
        goal_pub()
    except rospy.ROSInterruptException:
        pass
