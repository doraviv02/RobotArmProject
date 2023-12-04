import argparse

import pyrealsense2 as rs
import numpy as np
import cv2
from skimage.transform import ProjectiveTransform
import matplotlib.pyplot as plt
def start():
    # Open Camera Stream
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
    align_to = rs.stream.color
    align = rs.align(align_to)
    pipeline.start(config)
    src= [[0,0],[0,0],[0,0],[0,0]]

    try:
        while True:  # While recieving frames
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            aligned_depth_frame = aligned_frames.get_depth_frame()
            frames = pipeline.wait_for_frames()
            depth_image = frames.get_depth_frame()
            color_frame = frames.get_color_frame()

            if not color_frame and not depth_image:
                continue

            # Display depth Image
            # depth_image = np.asanyarray(aligned_depth_frame.get_data())
            # depth_image = (np.divide(depth_image,np.max(depth_image))*255).astype(np.uint8)
            # image = cv2.applyColorMap(depth_image, cv2.COLORMAP_PLASMA)
            # cv2.imshow("Test",image)

            # Convert the color frame to a numpy array
            color_image = np.asanyarray(color_frame.get_data())
            # Display the color image using OpenCV
            image_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
            arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            arucoParams = cv2.aruco.DetectorParameters()
            detector = cv2.aruco.ArucoDetector(arucoDict,arucoParams)
            (corners, ids, rejected) = detector.detectMarkers(image_rgb)


            # Corner order: 0- Top left, 1-top right, 2-bottom right, 3- bottom left (clockwise)
            if (np.array(ids).flatten().size ==4):
                aruco_id_index = [np.where((np.array(ids).flatten() == i))[0][0] for i in np.arange(0, 4, 1)]
                src = np.array([corners[aruco_id_index[2]][0][2], corners[aruco_id_index[3]][0][3],
                               corners[aruco_id_index[0]][0][0], corners[aruco_id_index[1]][0][1]])
                src = src.astype(int)  # Relevant coordinates

            # Projection of shape to rectangular image
            pts1 = np.float32([src[0], src[1], src[3], src[2]])
            pts2 = np.float32([[0, 0], [520, 0], [0, 260], [520, 260]])
            M = cv2.getPerspectiveTransform(pts1, pts2)
            dst = cv2.warpPerspective(image_rgb, M, (520, 260))

            # Hard Coded filter to detect tan and red blocks
            cv2.imshow("Red Channel", dst[:, :, 0])

            cv2.imshow("Transformation", dst)

            # edge image
            # edge_img = cv2.Canny(cv2.cvtColor(dst, cv2.COLOR_BGR2GRAY), 120, 150)
            # cv2.imshow("Edge Detection After Transformation", edge_img)

            # edge_before = cv2.Canny(cv2.cvtColor(image_rgb, cv2.COLOR_BGR2GRAY), 120, 150)
            # edge_dst = cv2.warpPerspective(edge_before, M, (300,300))
            # cv2.imshow("Edge Detection Before Transformation", edge_dst)

            # Draw Rectangle representing shape
            image_rgb = cv2.line(image_rgb, src[0], src[1], (0, 0, 255), 2)
            image_rgb = cv2.line(image_rgb, src[1], src[2], (0, 0, 255), 2)
            image_rgb = cv2.line(image_rgb, src[2], src[3], (0, 0, 255), 2)
            image_rgb = cv2.line(image_rgb, src[3], src[0], (0, 0, 255), 2)

            cv2.imshow("Color Frame", image_rgb)





            # Detect closing frame
            key = cv2.waitKey(1)
            if key == ord('q'):
                # Capturing Image for
                cv2.imwrite('output.jpg',dst)
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()




start()