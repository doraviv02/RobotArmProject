import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull
import cv2
from segmentation import segfunc

def find_centroids(dst):
    ret, dst = cv2.threshold(dst, 0.01 * dst.max(), 255, 0)
    dst = np.uint8(dst)

    # find centroids
    ret, labels, stats, centroids = cv2.connectedComponentsWithStats(dst)
    # define the criteria to stop and refine the corners
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100,
                0.001)
    corners = cv2.cornerSubPix(gray,np.float32(centroids),(5,5),
              (-1,-1),criteria)
    return corners

def up(points):
    #plt.figure()
    #plt.scatter(points[:, 0], points[:, 1])
    #plt.xlim([-1, 5])
    #plt.ylim([-1, 4])
    #plt.show()

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

    #lowest_point_idx = np.argmin(points[:,0])
    #orderred_indecies = ConvexHull(points).vertices
    #orderred_points = points[orderred_indecies]

    #left_to_lowest = (lowest_point_idx - 1) % len(points)
    #above_left = (left_to_lowest - 1) % len(points)

    #print(orderred_points)
    #offset = orderred_points[above_left] - orderred_points[left_to_lowest]
    #missing_point = orderred_points[lowest_point_idx] + offset

    #plt.figure()
    #plt.scatter(orderred_points[:, 0], orderred_points[:, 1])
    #plt.scatter(missing_point[0], missing_point[1])
    #plt.xlim([-1, 5])
    #plt.ylim([-1, 4])
    #plt.show()

    top_four = np.array([lost_corner, tl, top_mid, tr])

    return top_four

filename = 'test.png'
seg_blocks = segfunc(filename)

#running on each segmented block
for img in seg_blocks:
    # cv2.imshow('Original image', img)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = np.float32(gray)
    #dst = cv2.cornerHarris(gray, 2, 3, 0.04)
    corners = cv2.goodFeaturesToTrack(gray, 6, 0.1, 20)

    #corners = find_centroids(dst)
    #corners = np.delete(corners,0, axis=0)

    corners_coor = []

    #converting corner's dtype
    for corner in corners:
        x, y = corner[0]
        x_int, y_int = int(round(x)), int(round(y))
        corners_coor.append([x_int, y_int])

    # To draw the corners
    for corner in corners_coor:
       cv2.circle(img,(int(corner[0]), int(corner[1])), 5, (255,0,0), -1)

    cv2.imshow('Found Corners', img)

    points = corners_coor
    top = up(np.array(points))
    for point in top:
       cv2.circle(img,(int(point[0]), int(point[1])), 5, (0, 0, 255), -1)

    top_rs = top.reshape((-1, 1, 2)).astype(int)
    cv2.fillPoly(img, [top_rs], (0, 255, 0))

    cv2.imshow('Predicted top', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


