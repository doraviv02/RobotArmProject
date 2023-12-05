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

    mid_corners = np.delete(xtemp, [0, 1, 4, 5], axis=0)

    lowest_mid = mid_corners[np.argmax(mid_corners[:,1])]
    top_mid = mid_corners[np.argmin(mid_corners[:,1])]
    lost_corner = lowest_mid + (np.array(right_offset) + np.array(left_offset))/2

    top_four = np.array([lost_corner, tl, top_mid, tr])

    return top_four

def contours(blocks):
    contours = []
    for block in blocks:
        #prep contour
        contour, _ = cv2.findContours(block, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        contours.append(contour)

        cv2.drawContours(block, contour, -1, (0, 255, 0), 3)

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
def run(blocks):
    contour_list = contours(blocks)

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
    #running on each segmented block
    for poly in polys:
        corners_coor = []

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


        top = up(np.array(corners_coor))

        top_rs = top.reshape((-1, 1, 2)).astype(int)
        cv2.fillPoly(blank, [top_rs], (255, 255, 255))
        # centers.append(top.tolist())
        centers.append(top.tolist())

        cv2.imshow('Predicted top', blank)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    return(centers)
