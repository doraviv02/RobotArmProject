from xarm.wrapper import XArmAPI
import numpy as np
import cv2
import time

arm = XArmAPI('192.168.1.165')
initial_position = [200, 0, 200, 180, 0, 0]
arm.set_collision_rebound(on=False)
arm.set_collision_sensitivity(5, wait=True)


def complete_clean():
    arm.clean_error()
    arm.clean_gripper_error()
    arm.motion_enable(True)
    arm.set_state(state=0)
    time.sleep(0.5)


def move_down(rotation, pickup_height= None):
    cur = []
    position = arm.get_position_aa()[1]
    count = 0
    avg = 0
    position[2] = 130
    arm.set_position_aa(position, speed=50, wait=True)
    time.sleep(0.5)
    arm.set_servo_angle(servo_id=6, angle=rotation, is_radian=False, relative=True)
    if rotation != 0:
        time.sleep(3)
    position = arm.get_position_aa()[1]
    if (pickup_height != None):
      position[2] = pickup_height
      arm.set_position_aa(position, speed=50, wait=True)
      return True, pickup_height
    else:
      while not (np.linalg.norm(arm.currents) < 0.85 * avg and count > 10):
        # arm.set_position_aa([*start_pos, arm.get_position()[1][2]-5, 180, 0, 0], speed=20 ,wait=True)
        cur.append(np.linalg.norm(arm.currents))
        # position = arm.get_position_aa()[1]
        position[2] -= 2
        avg = np.mean(cur)

        print(np.linalg.norm(arm.currents))
        print("\n")
        arm.set_position_aa(position, speed=20, wait=True)
        count += 1
        if (position[2] < 65):
            arm.set_position_aa(initial_position, speed=50, wait=True)
            time.sleep(5)
            return False, None
      return True, position[2]+5


def stack_func(start_pos, end_pos, rotation):
    complete_clean()
    arm.set_position_aa([*start_pos, 200, 180, 0, 0], speed=50, mvacc=500, wait=True)
    # z=arm.get_position()[1][2]
    time.sleep(0.5)
    fail, pickup_height = move_down(0)
    if not fail:
        return

    # arm.config_cgpio_reset_when_stop(on_off=True)
    # arm.set_collision_rebound(on=False)

    complete_clean()
    #arm.reset()
    time.sleep(0.5)


    time.sleep(0.5)
    arm.set_vacuum_gripper(on=True, wait=True)

    arm.set_position_aa([*start_pos, 200, 180, 0, 0], speed=50, wait=True)
    arm.set_position_aa(initial_position, speed=50, wait=True)

    ft_status = arm.get_ft_sensor_config()

    arm.set_position_aa([*end_pos, 200, 180, 0, 0], speed=50, wait=True)
    fail, _ = move_down(rotation, pickup_height)
    if not fail:
        return
    complete_clean()

    # arm.set_position_aa([*end_pos, arm.get_position()[1][2]+5, 180, 0, 0], wait=True)
    time.sleep(0.5)
    arm.set_vacuum_gripper(False, wait=True)

    arm.set_position_aa([*end_pos, 200, 180, 0, 0], speed=50, wait=True)
    arm.set_position_aa(initial_position, speed=50, wait=True)
