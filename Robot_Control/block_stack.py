from xarm.wrapper import XArmAPI
import numpy as np
import cv2
import time

arm = XArmAPI('192.168.1.165')
initial_position = [200,0,200,180,0,0]
arm.set_collision_rebound(on=True)


def complete_clean():
  arm.clean_error()
  arm.clean_gripper_error()
  arm.motion_enable(True)
  arm.set_state(state=0)
  time.sleep(0.5)


def stack_func(start_pos, end_pos):

  complete_clean()
  arm.set_position_aa([*start_pos,200,180,0,0],wait=True)
  time.sleep(0.5)
  arm.set_position_aa([*start_pos, 0, 180, 0, 0], speed=20 ,wait=True)

  # complete_clean()
  #arm.config_cgpio_reset_when_stop(on_off=True)

  # arm.set_collision_rebound(on=False)
  complete_clean()
  arm.set_position_aa([*start_pos, arm.get_position()[1][2] + 10, 180, 0, 0], wait=True)

  # curr_position = arm.get_position()
  # curr_position[1][2] -= 10
  # arm.set_position_aa(curr_position[1],wait=True)
  time.sleep(0.5)
  arm.set_vacuum_gripper(on=True, wait=True)


  arm.set_position_aa([*start_pos,200,180,0,0],wait=True)
  arm.set_position_aa([*end_pos,200,180,0,0],wait=True)
  arm.set_position_aa([*end_pos,0,180,0,0],speed=20,wait=True)

  complete_clean()
  arm.set_position_aa([*end_pos, arm.get_position()[1][2]+10, 180, 0, 0], wait=True)
  time.sleep(0.5)


  arm.set_vacuum_gripper(False, wait=True)
  arm.set_position_aa([*end_pos,200,180,0,0],wait=True)
  arm.set_position_aa(initial_position,wait=True)




