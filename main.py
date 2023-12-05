import os
import shutil
import numpy as np
import cv2
import camera_control
import Segmentation.U_2_Net.u2net_test as u2net
import Segmentation.find_instances as find_instances
import Segmentation.top_finder as top_finder
import sys

from xarm.wrapper import XArmAPI

#arm = XArmAPI('COM5')
arm = XArmAPI('192.168.1.165')
arm.connect()

def find_robot_transformation():
    # top left, top right, bottom left, bottom right
    points = np.float32([[120.6, -250.9], [126, 251.5], [349.4, -255.6], [352.2, 251.2]])
    projected_size = [520, 1040]  # x and y are flipped in the robot
    border = np.float32([[0, 0], [1040, 0], [0, 520], [1040, 520]])
    return (cv2.getPerspectiveTransform(border, points))


run_again = True
if run_again:
    # Run Camera Module, Get the transformation matrix from camera to aruco outline
    M_camera = camera_control.start()

    # Run the U_2_Net model
    u2net.main()

# Separate Instances
blocks = find_instances.run("camera_output.jpg", "./Segmentation/camera_output.png")

corners = top_finder.run(blocks)
M_table_to_robot = find_robot_transformation()

# print(tops)
# for top in tops:
#     res = cv2.perspectiveTransform(top, M_table_to_robot)
#     print(res)
#
# cv2.waitKey(0)
# cv2.destroyAllWindows()
for set in corners:
    transformed_points = np.squeeze(cv2.perspectiveTransform(np.array(set).reshape(-1, 1, 2), M_table_to_robot))
    center_transformed = np.average(transformed_points, axis=0)
    center_transformed[0] = center_transformed[0]+25
    print(center_transformed)
    arm.set_position_aa([*center_transformed, 120, 180, 0, 0], wait=True)
print(corners)
initial_position = [200,0,200,180,0,0]
arm.set_position_aa(initial_position,wait=True)
