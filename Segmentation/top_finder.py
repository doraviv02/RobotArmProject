import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull
import cv2
import Segmentation.find_edges as find_edges

camera_h = 41  # cm


def up(points):
    xorder = np.argsort(points[:, 0])
    xtemp = points[xorder]

    right = np.array([xtemp[4], xtemp[5]])
    tr = right[np.argmin(right[:, 1])]
    br = right[np.argmax(right[:, 1])]

    left = np.array([xtemp[0], xtemp[1]])
    tl = left[np.argmin(left[:, 1])]
    bl = left[np.argmax(left[:, 1])]

    right_offset = tr - br
    left_offset = tl - bl
    mean_height_offset = np.mean([right_offset, left_offset], axis=0)

    mid_corners = np.delete(xtemp, [0, 1, 4, 5], axis=0)

    lowest_mid = mid_corners[np.argmax(mid_corners[:, 1])]
    top_mid = mid_corners[np.argmin(mid_corners[:, 1])]
    lost_corner = lowest_mid + mean_height_offset

    top_four = np.array([lost_corner, tl, top_mid, tr])

    # return top_four, np.mean([right_offset, left_offset])
    return top_four, mean_height_offset


def find_centroids(blocks):
    centroid_list = []
    for block in blocks:
        M = cv2.moments(block)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        centroid_list.append([cX, cY])
    return centroid_list


def pix2cam(center, M):
    center_in_robotsys = np.squeeze(cv2.perspectiveTransform(np.array(center, dtype=np.float32).reshape(-1, 1, 2), M))
    center_in_camsys = -np.array([center_in_robotsys[0] - 610, center_in_robotsys[1]])  # in mm
    return center_in_camsys


def contours(blocks):
    contours = []
    for block in blocks:
        contour, _ = cv2.findContours(block, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        contours.append(contour)

    return contours


def contour2poly(contour, num_points=6, is_open=True):
    epsilon = cv2.arcLength(contour, is_open)
    poly = cv2.approxPolyDP(contour, epsilon, is_open)
    count = 0
    while len(poly) != num_points:
        epsilon *= 0.9
        poly = cv2.approxPolyDP(contour, epsilon, is_open)
        if count == 100:
            break
        count += 1
    return poly


def find_bottom(blocks):
    new_blocks = []
    for block in blocks:
        new_block = block.copy()
        for y in range(block.shape[0] - 1):
            for x in range(block.shape[1]):
                if new_block[y + 1, x] != 0:
                    new_block[y, x] = 0
        new_blocks.append(new_block)
    return new_blocks


def angles(cam2center, leftvec, rightvec):
    nleftvec = leftvec / np.linalg.norm(leftvec)
    nrightvec = rightvec / np.linalg.norm(rightvec)
    ncam2center = cam2center / np.linalg.norm(cam2center)

    left_angle = np.dot(nleftvec, ncam2center)
    right_angle = np.dot(nrightvec, ncam2center)

    return left_angle, right_angle


def run(blocks, M):
    contour_list = contours(blocks)
    centroid_list = find_centroids(blocks)

    corners_list = []
    mean_height_offsets = []
    for idx in range(len(contour_list)):
        block = blocks[idx]
        contour_block = np.zeros_like(block)
        contour_block = cv2.drawContours(contour_block, contour_list[idx][0], -1, (255, 255, 255), 3)

        classification, line = find_edges.classify_box(contour_block, centroid_list[idx])

        if classification == 1:
            m, b = line[2], line[3]
            top_part, bottom_part = block.copy(), block.copy()

            top_contour, bottom_contour =  contour_list[idx][0].copy(), contour_list[idx][0].copy()

            for j in range(len(contour_list[idx][0])-1, -1, -1):
                point = contour_list[idx][0][j].squeeze()
                if (point[1] > m * point[0] + b):
                    top_contour = np.delete(top_contour, j, 0)
                elif (point[1] < m * point[0] + b):
                    bottom_contour = np.delete(bottom_contour, j, 0)

            top_contour = np.array(top_contour)
            bottom_contour = np.array(bottom_contour)

            poly_top = contour2poly(top_contour, 4, True)
            poly_bottom = contour2poly(bottom_contour, 4, True)

            corners_list.append(np.around(poly_top[:, 0]).astype(np.float64).tolist())
            bottom_points = np.sort(np.around(poly_bottom[:, 0]).astype(np.uint8))
            avg_vector = ((bottom_points[2]+bottom_points[3])-(bottom_points[0]+bottom_points[1]))*0.5
            mean_height_offsets.append(avg_vector)
            blank = np.zeros_like(block)
            # cv2.drawContours(blank, poly_top, -1, (255, 255, 255), 3)
            # cv2.drawContours(blank, poly_bottom, -1, (255,255, 255), 3)
            # cv2.imshow('First blank', blank)
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()


        elif classification == 2:
            poly = contour2poly(contour_list[idx][0])
            blank = np.zeros_like(block)
            cv2.drawContours(blank, poly, -1, (255, 255, 255), 3)
            cv2.imshow('First blank', blank)

            poly_corners = np.around(poly[:, 0]).astype(np.uint8)
            top, mean_height_offset = up(poly_corners)
            top_rs = top.reshape((-1, 1, 2)).astype(int)
            cv2.fillPoly(blank, [top_rs], (255, 255, 255))
            corners_list.append(top.tolist())
            mean_height_offsets.append(mean_height_offset)


    return (corners_list, mean_height_offsets)
