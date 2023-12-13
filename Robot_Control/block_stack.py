from xarm.wrapper import XArmAPI
import numpy as np
import cv2
import time

arm = XArmAPI('192.168.1.165')
initial_position = [200,0,200,180,0,0]
arm.set_collision_rebound(on=False)
arm.set_collision_sensitivity(5, wait=True)

def complete_clean():
  arm.clean_error()
  arm.clean_gripper_error()
  arm.motion_enable(True)
  arm.set_state(state=0)
  time.sleep(0.5)

def move_down(rotation):
  cur = []
  position = arm.get_position_aa()[1]
  count = 0
  avg = 0
  position[2] = 130
  arm.set_position_aa(position, speed=30, wait=True)
  time.sleep(0.5)
  arm.set_servo_angle(servo_id=6, angle=rotation, is_radian=False, relative=True)
  if rotation != 0 :
    time.sleep(5)
  position = arm.get_position_aa()[1]
  while not (np.linalg.norm(arm.currents) < 0.85 * avg and count > 10):

    # arm.set_position_aa([*start_pos, arm.get_position()[1][2]-5, 180, 0, 0], speed=20 ,wait=True)
    cur.append(np.linalg.norm(arm.currents))
    #position = arm.get_position_aa()[1]
    position[2] -= 2
    avg = np.mean(cur)
    # if (count > 20):
    #   if (np.linalg.norm(arm.currents) < 0.8 * avg):
    #     print("done frfr")
    #     break
    print(np.linalg.norm(arm.currents))
    print("\n")
    arm.set_position_aa(position, speed=10 ,wait=True)
    count += 1

def stack_func(start_pos, end_pos, rotation):

  complete_clean()
  arm.set_position_aa([*start_pos,200,180,0,0],speed=30, wait=True)
  # z=arm.get_position()[1][2]
  time.sleep(0.5)
  move_down(0)

  #arm.config_cgpio_reset_when_stop(on_off=True)
  # arm.set_collision_rebound(on=False)
  complete_clean()
  time.sleep(0.5)
  #arm.set_position_aa([*start_pos, arm.get_position()[1][2]+5, *angles],speed=30, wait=True)


  # curr_position = arm.get_position()
  # curr_position[1][2] -= 10
  # arm.set_position_aa(curr_position[1],wait=True)
  time.sleep(0.5)
  arm.set_vacuum_gripper(on=True, wait=True)

  arm.set_position_aa([*start_pos,200,180,0,0],speed=30, wait=True)
  arm.set_position_aa(initial_position, speed=30, wait=True)


  arm.set_position_aa([*end_pos,200,180,0,0],wait=True)
  move_down(45)
  complete_clean()

  #arm.set_position_aa([*end_pos, arm.get_position()[1][2]+5, 180, 0, 0], wait=True)
  time.sleep(0.5)
  arm.set_vacuum_gripper(False, wait=True)

  arm.set_position_aa([*end_pos,200,180,0,0], speed=30, wait=True)
  arm.set_position_aa(initial_position, speed=30, wait=True)
