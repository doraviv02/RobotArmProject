from xarm.wrapper import XArmAPI
import numpy as np
import cv2
import time
from camera_control import transform_image
from skimage.metrics import structural_similarity as ssim

arm = XArmAPI('192.168.1.165')
initial_position = [200, 0, 200, 180, 0, 0]
arm.set_collision_rebound(on=False)
arm.set_collision_sensitivity(5, wait=True)

def pickup_check(row_threshold, cols_threshold):

    first_image = cv2.imread('camera_output_table.jpg')
    transform_image(None, 0) # Getting camera output at pickup
    pickup_image = cv2.imread('camera_output_pickup.jpg')

    offset = 150
    first_image = first_image[0:row_threshold,cols_threshold[0]+offset:cols_threshold[1]-offset]
    pickup_image = pickup_image[0:row_threshold, cols_threshold[0]+offset:cols_threshold[1]-offset]

    # cv2.imshow("first image", first_image)
    # cv2.imshow("second image", pickup_image)
    # cv2.waitKey()
    # cv2.destroyAllWindows()

    error = ssim(first_image,pickup_image,channel_axis=2)
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


def move_down(rotation, pickup_height=None):
    cur = []
    count = 0
    avg = 0

    position = arm.get_position_aa()[1]
    if pickup_height is not None:
        position[2] = pickup_height
        arm.set_position_aa(position, speed=30, wait=True)
        return True, pickup_height
    else:
        while not (np.linalg.norm(arm.currents) < 0.85 * avg and count > 10):
            # arm.set_position_aa([*start_pos, arm.get_position()[1][2]-5, 180, 0, 0], speed=20 ,wait=True)
            cur.append(np.linalg.norm(arm.currents))
            # position = arm.get_position_aa()[1]
            position[2] -= 2
            avg = np.mean(cur)

            #print(np.linalg.norm(arm.currents))
            #print("\n")
            arm.set_position_aa(position, speed=30, wait=True)

            count += 1
            if (position[2] < 65):
                arm.set_position_aa(initial_position, speed=30, wait=True)
                time.sleep(5)

                if (arm.get_position_aa()[2] > 65):
                    return None, None

                return False, None
        return True, position[2] + 5


def stack_func(start_pos, end_pos, rotation, row_threshold, cols_threshold):
    try:
        complete_clean()
        arm.set_position_aa([*start_pos, 130, 180, 0, 0], speed=70, mvacc=500, wait=True)
        # z=arm.get_position()[1][2]
        time.sleep(0.5)
        fail, pickup_height = move_down(0)
        if not fail:
            return 0

        complete_clean()

        time.sleep(0.5)
        arm.set_vacuum_gripper(on=True, wait=True)

        arm.set_position_aa([*start_pos, 130, 180, 0, 0], speed=70, wait=True)
        arm.set_position_aa(initial_position, speed=70, wait=True)

        # Check for success in picking up block
        success = pickup_check(row_threshold, cols_threshold)
        if not success:
            return 0

        arm.set_position_aa([*end_pos, 130, 180, 0, 0], speed=70, wait=True)
        arm.set_servo_angle(servo_id=6, angle=rotation, is_radian=False, relative=True, wait=True)

        print(arm.get_position_aa())
        fail, _ = move_down(rotation, pickup_height)
        if not fail:
            return 0
        complete_clean()

        # arm.set_position_aa([*end_pos, arm.get_position()[1][2]+5, 180, 0, 0], wait=True)
        time.sleep(0.5)
        arm.set_vacuum_gripper(False, wait=True)

        arm.set_position_aa([*end_pos, 130, 180, 0, 0], speed=70, wait=True)
        arm.set_position_aa(initial_position, speed=70, wait=True)

        return success
    except Exception:
        return None
