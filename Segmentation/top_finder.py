import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull
import cv2

# def find_centroids(dst):
#     ret, dst = cv2.threshold(dst, 0.01 * dst.max(), 255, 0)
#     dst = np.uint8(dst)
#
#     # find centroids
#     ret, labels, stats, centroids = cv2.connectedComponentsWithStats(dst)
#     # define the criteria to stop and refine the corners
#     criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)
#     corners = cv2.cornerSubPix(gray,np.float32(centroids), (5, 5), (-1, -1), criteria)
#     return corners
camera_h = 41 #cm
def up(points):

    xorder = np.argsort(points[:,0])
    xtemp = points[xorder]

    right = np.array([xtemp[4], xtemp[5]])
    tr = right[np.argmin(right[:, 1])]
    br = right[np.argmax(right[:, 1])]

    left = np.array([xtemp[0], xtemp[1]])
    tl = left[np.argmin(left[:, 1])]
    bl = left[np.argmax(left[:, 1])]

    right_offset = tr - br
    left_offset = tl - bl
    mean_offset = np.mean([right_offset, left_offset], axis=0)

    mid_corners = np.delete(xtemp, [0, 1, 4, 5], axis=0)

    lowest_mid = mid_corners[np.argmax(mid_corners[:,1])]
    top_mid = mid_corners[np.argmin(mid_corners[:,1])]
    lost_corner = lowest_mid + mean_offset

    top_four = np.array([lost_corner, tl, top_mid, tr])

    # return top_four, np.mean([right_offset, left_offset])
    return top_four, mean_offset
def pix2cam(center, M):
    center_in_robotsys = np.squeeze(cv2.perspectiveTransform(np.array(center, dtype=np.float32).reshape(-1, 1, 2), M))
    center_in_camsys = -np.array([center_in_robotsys[0]-610, center_in_robotsys[1]])#in mm
    return center_in_camsys
def contours(blocks):
    contours = []
    for block in blocks:
        #prep contour
        contour, _ = cv2.findContours(block, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        contours.append(contour)
        #blank = np.zeros_like(blocks[0])
        #cv2.drawContours(blank, contour, -1, (255, 255, 255), 3)

    return contours
def test():
    contours()
def contour2poly(contours, blocks_shape):
    polys = []

    for contour in contours:
        epsilon = cv2.arcLength(contour[0], True)
        poly = cv2.approxPolyDP(contour[0], epsilon, True)
        count = 0
        while len(poly)!= 6:
            epsilon*=0.9
            poly = cv2.approxPolyDP(contour[0], epsilon, True)
            if count == 100:
                break
            count += 1

        blank = np.zeros(blocks_shape)
        cv2.drawContours(blank, [poly], -1, (255, 255, 255), 3)
        # cv2.imshow('testing', blank)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        polys.append(poly)

    return polys
def find_bottom(blocks):
    new_blocks = []
    for block in blocks:
        new_block = block.copy()
        for y in range(block.shape[0]-1):
            for x in range(block.shape[1]):
                if new_block[y+1,x] != 0:
                    new_block[y, x] = 0
        new_blocks.append(new_block)
    return new_blocks

def angles(cam2center, leftvec, rightvec):
    nleftvec = leftvec/np.linalg.norm(leftvec)
    nrightvec = rightvec / np.linalg.norm(rightvec)
    ncam2center = cam2center / np.linalg.norm(cam2center)

    left_angle = np.dot(nleftvec, ncam2center)
    right_angle = np.dot(nrightvec, ncam2center)

    return left_angle, right_angle

def run(blocks, M):
    contour_list = contours(blocks)
    block_bottoms_list = find_bottom(blocks)
    leftvec_list = []
    rightvec_list = []

    for block_bottom in block_bottoms_list:
        filter= np.ones((3,3), np.uint8)/9
        new_block_bottom = cv2.filter2D(block_bottom, -1, filter)
        new_block_bottom[new_block_bottom > 255/9] = 255
        new_block_bottom[new_block_bottom <= 255 / 9] = 0
        block_bottom = cv2.bitwise_and(block_bottom, new_block_bottom)
        #block_bottom = cv2.GaussianBlur(block_bottom, (3, 3), 0)
        #block_bottom = cv2.bilateralFilter(block_bottom, 3 , 150, 150)
        #block_bottom = cv2.morphologyEx(block_bottom, cv2.MORPH_CLOSE, np.ones((3,3),np.uint8), iterations=1)
        #block_bottom = cv2.morphologyEx(block_bottom, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8), iterations=1)
        cv2.imshow('block_bottom', block_bottom)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        valid_indexes = np.where(block_bottom != 0)
        bottom_idx = np.argmax(valid_indexes[0])
        left_idx = np.argmin(valid_indexes[1])
        right_idx = np.argmax(valid_indexes[1])

        bottom_point = np.array([valid_indexes[0][bottom_idx], valid_indexes[1][bottom_idx]])
        left_point = np.array([valid_indexes[0][left_idx], valid_indexes[1][left_idx]])
        right_point = np.array([valid_indexes[0][right_idx], valid_indexes[1][right_idx]])

        left_point = pix2cam(left_point,M)
        right_point = pix2cam(right_point,M)
        bottom_point = pix2cam(bottom_point,M)

        left_vector = left_point-bottom_point
        right_vector = right_point-bottom_point

        leftvec_list.append(left_vector)
        rightvec_list.append(right_vector)
    con_img = []
    for contour in contour_list:
        blank = np.zeros_like(blocks[0])
        cv2.drawContours(blank, contour, -1, (255, 255, 255), 2)
        # blank = cv2.cvtColor(blank, cv2.COLOR_BGR2GRAY)
        con_img.append(blank)
        cv2.imshow('testing', blank)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    polys = contour2poly(contour_list, np.array(blocks[0]).shape)
    centers = []
    means = []
    #running on each segmented block
    for poly in polys:
        corners_coor = []
        i = 0
        #converting corner's dtype
        for corner in poly:
            x, y = corner[0]
            x_int, y_int = int(round(x)), int(round(y))
            corners_coor.append([x_int, y_int])

        # To draw the corners
        blank = np.zeros_like(blocks[0])
        cv2.drawContours(blank, poly, -1, (255, 255, 255), 3)
        # blank = cv2.cvtColor(blank, cv2.COLOR_BGR2GRAY)
        cv2.imshow('First blank', blank)

        top, mean_offset = up(np.array(corners_coor))

        top_rs = top.reshape((-1, 1, 2)).astype(int)
        cv2.fillPoly(blank, [top_rs], (255, 255, 255))
        # centers.append(top.tolist())

        cam2center = pix2cam(np.mean(top, axis=0),M)

        left_cos, right_cos = angles(cam2center, leftvec_list[i], rightvec_list[i])
        left_angle = np.arccos(left_cos)*180/np.pi
        right_angle = np.arccos(right_cos) * 180 / np.pi

        i += 1

        centers.append(top.tolist())
        means.append(mean_offset)
        cv2.imshow('Predicted top', blank)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    return(centers, means)
