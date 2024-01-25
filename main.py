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

#arm = XArmAPI('COM5')
arm = XArmAPI('192.168.1.165')
arm.connect()


def find_robot_transformation():
    # top left, top right, bottom left, bottom right
    points = np.float32([[120.6, -250.9], [126, 251.5], [349.4, -255.6], [352.2, 251.2]])
    projected_size = [520, 1040]  # x and y are flipped in the robot
    border = np.float32([[0, 0], [1040, 0], [0, 520], [1040, 520]])
    return (cv2.getPerspectiveTransform(border, points))


def main():
    # Run Camera Module, Get the transformation matrix from camera to aruco outline
    #M, top_points = camera_control.detect_aruco()
    #M_table_to_robot = find_robot_transformation()
    #M = camera_control.start()

    while True:

        # Save transformed image
        camera_control.transform_image(M, True)

        # Run the U_2_Net model
        u2net.main()

        # Separate Instances
        blocks = find_instances.run("camera_output.jpg", "./Segmentation/camera_output.png")

        corners,h_means = top_finder.run(blocks,M_table_to_robot)


        centers = []
        for set in range(len(corners)):
            center = np.average(corners[set], axis=0) - h_means[set]
            transformed_points = np.squeeze(cv2.perspectiveTransform(np.array(center).reshape(-1, 1, 2), M_table_to_robot))
            #transformed_points = np.squeeze(cv2.perspectiveTransform(np.array(center).reshape(-1, 1, 2), M_table_to_robot))
            #center_transformed = np.average(transformed_points, axis=0)
            # center_transformed[0] +=  0.40 * length
            # center_transformed[1] +=  0.20 * h_means[set][0]
            centers.append(transformed_points)

        flag_close = []
        while True:
            randx = np.random.randint(200, 300)
            randy = np.random.randint(-170, 170)
            randrz = np.random.randint(0, 90)

            for center in centers[1:]:
                flag_close.append((np.abs(randx-center[0]) > 30) and (np.abs(randy-center[1]) > 30))
            # for i in range(1, len(centers)):
            #     #print("randx: " + str(randx) + " center x : " + str(centers[i][0]))
            #     flag_close.append( (np.abs(randx-centers[i][0]) > 45) and (np.abs(randy-centers[i][1]) > 70))
            if np.any(flag_close) or (len(centers) == 1):
                break

        rand_place = np.array([randx, randy])

        row_threshold = int(np.average([top_points[0][1], top_points[1][1]]))
        cols_threshold = np.array([int(top_points[0][0]), int(top_points[1][0])])

        is_picked = block_stack.stack_func(centers[0], rand_place, randrz, row_threshold, cols_threshold)
        if (is_picked is not None):
            success.append(int(is_picked))
            np.savetxt('data_'+current_Time+'.csv', np.array(success),  fmt='%i', delimiter=',')

        block_stack.complete_clean()
        block_stack.set_initial_position()

current_Time = str(datetime.now(timezone.utc))[:-13]
success = []

M, top_points = camera_control.detect_aruco()
M_table_to_robot = find_robot_transformation()

while (True):
    block_stack.complete_clean()
    block_stack.set_initial_position()
    try:
        main()
    except Exception:
        #TODO: add clean errors and such
        # block_stack.complete_clean()
        # block_stack.set_initial_position()
        pass