from turtle import delay
import rospy
from frankapy import FrankaArm

if __name__ == '__main__':
    fa = FrankaArm()
    fa.close_gripper()
    while(1):
        force_torque = fa.get_ee_force_torque()
        print(force_torque)
        delay(500)