# Write a node that publishes a goal pose to the /goal_location topic as a geometry_msgs/PoseStamped message
import rospy
from geometry_msgs.msg import PoseStamped

def goal_pub():
    pub = rospy.Publisher('/goal_location', PoseStamped, queue_size=10)
    rospy.init_node('goal_pub', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    goal = PoseStamped()
    goal.pose.position.y = -0.25791107711908864
    temp = 1
    while not rospy.is_shutdown():
        # Keep incerementing the y position of the goal pose by 0.1
        goal.header.seq = 0
        goal.header.stamp = rospy.Time.now()        
        goal.header.frame_id = ""
        goal.pose.position.x = 0.3843781940153249
        goal.pose.position.y += temp*0.1
        goal.pose.position.z = 0.23098061041636195
        goal.pose.orientation.x = -0.9186984147774666
        goal.pose.orientation.y = 0.3942492534293267
        goal.pose.orientation.z = -0.012441904611284204
        goal.pose.orientation.w = 0.020126567105018894 
        # Keep doing this until the y position of the goal pose reaches 0.3 then stop the node.
        if goal.pose.position.y >= 0.29 or goal.pose.position.y <= -0.24:
            temp = temp*(-1)
        rospy.loginfo(goal)
        pub.publish(goal)
        rate.sleep()
        

if __name__ == '__main__':
    try:
        goal_pub()
    except rospy.ROSInterruptException:
        pass
