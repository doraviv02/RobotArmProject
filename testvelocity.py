import os
import shutil
import numpy as np
import cv2
import camera_control
import Segmentation.U_2_Net.u2net_test as u2net
import Segmentation.find_instances as find_instances
import Segmentation.top_finder as top_finder
import Robot_Control.block_stack as block_stack
import sys
from datetime import datetime, timezone
from xarm.wrapper import XArmAPI
from time import sleep
import time


#arm = XArmAPI('COM5')
arm = XArmAPI('192.168.1.165')
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)
time.sleep(1)


print(arm.version)

#arm.reset(wait=True)

# set cartesian velocity control mode
arm.set_mode(5)
arm.set_state(0)
time.sleep(1)

cur = []
count = 0
avg = 0

arm.set_collision_sensitivity(5, wait=True)
#ft = arm.joints_torque
arm.vc_set_cartesian_velocity([0, 0, -40, 0, 0, 0])

arm.set_vacuum_gripper(on=True)
while not (arm.get_suction_cup()[1]):
#while not (np.abs(arm.joints_torque[1]) < 1.05*avg and count > 30):
    #time.sleep(0.05)
    #arm.vc_set_cartesian_velocity([0, 0, -1, 0, 0, 0])
    # arm.set_position_aa([*start_pos, arm.get_position()[1][2]-5, 180, 0, 0], speed=20 ,wait=True)
    cur.append(np.linalg.norm(arm.currents))
    # position = arm.get_position_aa()[1]
    avg = np.mean(cur[-10:])
    print(count)
    count += 1
    print('currents now:',  np.linalg.norm(arm.currents))
    print(arm.robotiq_status)

    warn_code = (arm.get_err_warn_code())[1][0]
    #print(warn_code)
    #print(arm.get_err_warn_code())

    if (warn_code == 31 or warn_code == 35):
        print("Warning: Overload")
        break

arm.set_vacuum_gripper(on=False)
# time.sleep(2)
# arm.vc_set_cartesian_velocity([0, 50, 0, 0, 0, 0])
# time.sleep(2)
# arm.vc_set_cartesian_velocity([0, 0, 50, 0, 0, 0])
# time.sleep(2)
# arm.vc_set_cartesian_velocity([0, 50, 0, 0, 0, 0])
# time.sleep(4)
# arm.vc_set_cartesian_velocity([0, 0, -50, 0, 0, 0])
# time.sleep(2)
# stop
arm.vc_set_cartesian_velocity([0, 0, 0, 0, 0, 0])