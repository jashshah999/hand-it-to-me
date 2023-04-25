from gettext import translation
import imp
from turtle import delay
import rospy
import numpy as np
from autolab_core import RigidTransform
from frankapy import FrankaArm

if __name__ == '__main__':
    fa = FrankaArm()
    # fa.close_gripper()
    # print("Starting Guide Mode")
    # fa.run_guide_mode(30)
    # print("End Guide Mode")
    des_pose = RigidTransform(rotation=np.array([[1,0,0],
                                                [0,-1,0],
                                                [0,0,-1]]), 
                            translation=np.array([0.7424810437698631, 0.3305789505306479, 0.34548961]),
                            from_frame='franka_tool', to_frame='world')
    fa.goto_pose(des_pose, use_impedance=False)
    # while(1):
    #     force_torque = fa.get_ee_force_torque()
    #     print(force_torque)
    #     delay(500)