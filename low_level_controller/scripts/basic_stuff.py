from gettext import translation
from queue import Empty
from xml.etree.ElementInclude import include
import numpy as np
from autolab_core import RigidTransform
from frankapy import FrankaArm
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from scipy.spatial.transform import Rotation as R




class Controller:
		def __init__(self) -> None:
			self.fa = FrankaArm()
			self.robot_pose_pub = rospy.Publisher('robot_pose', PoseStamped, queue_size=10)
			self.robot_force_pub = rospy.Publisher('force_on_gripper', Float32, queue_size=10)
			self.gripper_width_pub = rospy.Publisher('gripper_width', Float32, queue_size=10)
			self.get_goal_pose = rospy.Subscriber("goal_pose", PoseStamped, self.go_to_goal)
			self.go_to_home_srv = rospy.Service('go_to_home', Empty, self.reset)
			self.grab_object_srv = rospy.Service('pickup', Empty, self.grab_obj)
			self.handover_srv = rospy.Service('hand_over_object', Empty, self.open_gripper)

		def do_stuff(self):
			robot_pose = self.fa.get_pose()
			quat = R.from_matrix(robot_pose.rotation)
			quat = quat.as_quat()
			trans = robot_pose.translation
			posestamped = PoseStamped()
			posestamped.pose.position.x = trans[0]
			posestamped.pose.position.y = trans[1]
			posestamped.pose.position.z = trans[2]
			posestamped.pose.orientation.x = quat[0]
			posestamped.pose.orientation.y = quat[1]
			posestamped.pose.orientation.z = quat[2]
			posestamped.pose.orientation.w = quat[3]
			force_on_gripper = self.fa.get_ee_force_torque()
			force_on_gripper = np.sqrt(force_on_gripper[0]**2 + force_on_gripper[1]**2 + force_on_gripper[2]**2)
			gripper_width = self.fa.get_gripper_width()
			self.robot_pose_pub.publish(posestamped)
			self.robot_force_pub.publish(force_on_gripper)
			self.gripper_width_pub.publish(gripper_width)

		def reset(self, req):
			self.fa.reset_joints()       
			return True

		def grab_obj(self, req):
			self.fa.goto_gripper(0.1)
			return True

		def open_gripper(self, req):
			self.fa.open_gripper()
			return True

		def go_to_goal(self, msg):
			quat = msg.pose.orientation
			r = R.from_quat([quat])
			t = msg.pose.position
			des_pos = RigidTransform(rotation=np.array(r.as_matrix()), translation=np.array(t),
						from_frame='franka_tool', to_frame='world')
			self.fa.goto_pose(des_pos, duration=1.0, use_impedance=False, cartesian_impedances=[3000, 3000, 100, 300, 300, 300])



if __name__ == "__main__":

	# rospy.init_node('low_level_controller')
	controller = Controller() 
	rate = rospy.Rate(10) # 10hz

	# rospy.spin()
	while not rospy.is_shutdown():
		controller.do_stuff()
		rate.sleep()
	
