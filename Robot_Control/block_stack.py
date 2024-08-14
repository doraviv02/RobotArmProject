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

    first_image = cv2.imread('camera_output_table.jpg')
    transform_image(None, 0) # Getting camera output at pickup
    pickup_image = cv2.imread('camera_output_pickup.jpg')

    offset = 150
    first_image = first_image[0:row_threshold, cols_threshold[0]+offset:cols_threshold[1]-offset]
    pickup_image = pickup_image[0:row_threshold, cols_threshold[0]+offset:cols_threshold[1]-offset]

    # cv2.imshow("first image", first_image)
    # cv2.imshow("second image", pickup_image)
    # cv2.waitKey()
    # cv2.destroyAllWindows()

    error = ssim(first_image, pickup_image, channel_axis=2)
    print(error)
    thresh = 0.9


    is_picked = (error <= thresh)

    if not is_picked:
        arm.set_vacuum_gripper(on=False, wait=True)

    return is_picked


def complete_clean():
    arm.clean_error()
    arm.clean_gripper_error()
    arm.motion_enable(True)
    arm.set_state(state=0)
    time.sleep(0.5)


def set_initial_position():
    arm.set_position_aa(initial_position, speed=70, wait=True)


def move_down(rotation):
    cube_height = 85.1 #[mm]
    position = arm.get_position_aa()[1]
    # if pickup_height is not None:
    #position[2] = pickup_height
    #     arm.set_position_aa(position, speed=30, wait=True)
    #     return True, pickup_height
    # else:
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

    # arm.set_mode(5)
    # arm.set_state(0)
    # arm.set_collision_sensitivity(5)
    # time.sleep(0.1)
    #arm.vc_set_cartesian_velocity([0, 0, -10, 0, 0, 0])
    # timeout = time.time() + 5
    # prev_position = [0, 0, 100000] # initial placeholder
    # position = [0,0,6600] #placeholder
    # location = open("location.txt","a")
    # diff_arr = []
    # while (not(arm.get_suction_cup()[1]) and prev_position[2]- position[2] > 0.0001):
    #     location.write(str(position[2])+" "+str(prev_position[2]))
    #     location.write("\n")
    #     if position[2] < 65: # Checking if close to ground
    #         arm.set_position_aa(initial_position, speed=30, wait=True)
    #         time.sleep(3)
    #         if position[2] > 65: # was able to get back up
    #             return None, None
    #         return False, None
    #     prev_position = position
    #     time.sleep(0.01)
    #     position = arm.get_position_aa()[1]
    #     diff_arr.append(prev_position[2]-position[2])
    #
    #     print("position: " + str(prev_position[2]), "new position: " + str(position[2]))
    #     print(prev_position[2])
    #     print(position[2])
    #print("Were Out: "+ str(arm.get_suction_cup()[1]))
    #plt.figure(); plt.plot(diff_arr[2:]); plt.title("difference in z location"); plt.show()

    #arm.set_vacuum_gripper(on=False)

        # while not(arm.get_suction_cup()[1]):
        #     print(arm.get_suction_cup()[1])
        #     if position[2] < 65:
        #         arm.set_position_aa(initial_position, speed=30, wait=True)
        #         time.sleep(3)
        #         if arm.get_position_aa()[2] > 65:
        #             return None, None
        #         return False, None
        #
        # position = arm.get_position_aa()[1]
        # time.sleep(0.1)
        # position2 = arm.get_position_aa()[1]
        # arm.vc_set_cartesian_velocity([0, 0, 0, 0, 0, 0])


def stack_func(start_pos, end_pos, rotation, row_threshold, cols_threshold):
    try:
        complete_clean()
        arm.set_position_aa([*start_pos, 110, 180, 0, 0], speed=70, mvacc=500, wait=True)
        # z=arm.get_position()[1][2]
        time.sleep(0.5)
        #arm.set_vacuum_gripper(on=True)
        picked = move_down(0)
        if not picked:
            arm.set_vacuum_gripper(on=False)
            return 0

        complete_clean()

        time.sleep(0.5)
        #arm.set_vacuum_gripper(on=True, wait=True)

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
