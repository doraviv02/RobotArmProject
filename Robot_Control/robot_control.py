from xarm.wrapper import XArmAPI
import numpy as np
import cv2
import time
from camera_control import transform_image
from skimage.metrics import structural_similarity as ssim
import matplotlib.pyplot as plt

arm = XArmAPI('192.168.1.165')
initial_position = [200, 0, 200, 180, 0, 0]
arm.set_collision_rebound(on=False)
arm.set_collision_sensitivity(5, wait=True)


def pickup_check(row_threshold, cols_threshold):
    # function to check if there was a successful pickup based on 
    # the camera output before and after.
    # NOT USED IN FINAL IMPLEMENTATION
    first_image = cv2.imread('camera_output_table.jpg')
    transform_image(None, 0) # Getting camera output at pickup
    pickup_image = cv2.imread('camera_output_pickup.jpg')

    offset = 150
    first_image = first_image[0:row_threshold, cols_threshold[0]+offset:cols_threshold[1]-offset]
    pickup_image = pickup_image[0:row_threshold, cols_threshold[0]+offset:cols_threshold[1]-offset]

    error = ssim(first_image, pickup_image, channel_axis=2)
    print(error)
    thresh = 0.9


    is_picked = (error <= thresh)

    if not is_picked:
        arm.set_vacuum_gripper(on=False, wait=True)

    return is_picked


def complete_clean():
    # clean all errors for correct run
    arm.clean_error()
    arm.clean_gripper_error()
    arm.motion_enable(True)
    arm.set_state(state=0)
    time.sleep(0.5)


def set_initial_position():
    # set the robot to the initial position
    arm.set_position_aa(initial_position, speed=70, wait=True)


def move_down(rotation):
    # NOT USED IN FINAL IMPLEMENTATION`
    # function that dynamically stops while moving down and checking vacuum gripper
    # fault is reliability issue
    cube_height = 85.1 #[mm]
    position = arm.get_position_aa()[1]

    if rotation==0:
        arm.set_position_aa([*position[:2], cube_height+1, 180, 0, 0], speed=70, mvacc=500, wait=True)
        arm.set_vacuum_gripper(on=True)
        time.sleep(0.5)
        if arm.get_suction_cup()[1]:
            return True
        else:
            return False

    else:
        arm.set_position_aa([*position[:2], cube_height, 180, 0, 0], speed=70, mvacc=500, wait=True)
        arm.set_servo_angle(servo_id=6, angle=rotation, is_radian=False, relative=True, wait=True)
        arm.set_vacuum_gripper(on=False)
        return True


def pick_place(start_pos, end_pos, rotation, row_threshold, cols_threshold):
# perform the pick and place operation

    try:
        complete_clean()
        arm.set_position_aa([*start_pos, 110, 180, 0, 0], speed=70, mvacc=500, wait=True)
        time.sleep(0.5)
        picked = move_down(0)
        if not picked:
            arm.set_vacuum_gripper(on=False)  # turning off vacuum gripper
            return 0

        complete_clean()

        time.sleep(0.5)

        arm.set_position_aa([*start_pos, 180, 180, 0, 0], speed=70, wait=True)
        arm.set_position_aa([*end_pos, arm.get_position()[1][2]+5, 180, 0, 0],speed=60,wait=True)

        _ = move_down(rotation)
        complete_clean()

        time.sleep(0.5)

        arm.set_position_aa([*end_pos, 160, 180, 0, 0], speed=70, wait=True)
        arm.set_position_aa(initial_position, speed=70, wait=True)

        return True
    except Exception:
        return None
