import pyrealsense2 as rs
import numpy as np
import cv2


def calib(img):
    # Not used in final implementation
    # Camera calibration
    dist = np.load('./Camera_Calibration/dist.npy')
    mtx = np.load('./Camera_Calibration/mtx.npy')
    h, w = img.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))

    # undistort
    dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
    # crop the image
    x, y, w, h = roi
    dst = dst[y:y + h, x:x + w]
    return dst


def detect_aruco():
    # Open Camera Stream
    pipeline = rs.pipeline()
    config = rs.config()
    # config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 15)
    align_to = rs.stream.color
    align = rs.align(align_to)
    pipeline.start(config)
    src = [[0, 0], [0, 0], [0, 0], [0, 0]]

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

            # Convert the color frame to a numpy array
            color_image = np.asanyarray(color_frame.get_data())
            # Display the color image using OpenCV
            image_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
            cv2.imshow("Color Frame", image_rgb)
            key = cv2.waitKey(1)
            # image_rgb = calib(image_rgb)
            arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            arucoParams = cv2.aruco.DetectorParameters()
            detector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)
            (corners, ids, rejected) = detector.detectMarkers(image_rgb)

            # Corner order: 0- Top left, 1-top right, 2-bottom right, 3- bottom left (clockwise)
            if (np.array(ids).flatten().size == 4):
                aruco_id_index = [np.where((np.array(ids).flatten() == i))[0][0] for i in np.arange(0, 4, 1)]
                src = np.array([corners[aruco_id_index[2]][0][2], corners[aruco_id_index[3]][0][3],
                                corners[aruco_id_index[0]][0][0], corners[aruco_id_index[1]][0][1]])
                src = src.astype(int)  # Relevant coordinates

                # Draw Rectangle representing shape
                image_rgb = cv2.line(image_rgb, src[0], src[1], (0, 0, 255), 2)
                image_rgb = cv2.line(image_rgb, src[1], src[2], (0, 0, 255), 2)
                image_rgb = cv2.line(image_rgb, src[2], src[3], (0, 0, 255), 2)
                image_rgb = cv2.line(image_rgb, src[3], src[0], (0, 0, 255), 2)

                cv2.imshow("Color Frame", image_rgb)

                # Detect closing frame
                key = cv2.waitKey(1)

                # Projection of shape to rectangular image
                pts1 = np.float32([src[0], src[1], src[3], src[2]])
                pts2 = 2 * np.float32([[0, 0], [520, 0], [0, 260], [520, 260]])
                M = cv2.getPerspectiveTransform(pts1, pts2)
                # Draw Rectangle representing shape
                break


    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        return M, [pts1[0],pts1[1]]


def transform_image(M, first):
    # Open Camera Stream and get RGB image
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 15)
    pipeline.start(config)
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    color_image = np.asanyarray(color_frame.get_data())
    image_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)

    if first: # save the image before picking up the block
        cv2.imwrite('camera_output_table.jpg', image_rgb)
    else: # save image after picking up the block, to check if successful
        cv2.imwrite('camera_output_pickup.jpg', image_rgb)
        return

    # save the transformed image to be used in the U_2_Net model
    dst = cv2.warpPerspective(image_rgb, M, (2 * 520, 2 * 260))
    cv2.imwrite('camera_output.jpg', dst)
