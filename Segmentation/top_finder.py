import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull
import cv2

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
    plt.figure()
    plt.scatter(points[:, 0], points[:, 1])
    plt.xlim([-1, 5])
    plt.ylim([-1, 4])
    plt.show()

    lowest_point_idx = np.argmin(points[:, 0])
    orderred_indecies = ConvexHull(points).vertices
    orderred_points = points[orderred_indecies]

    left_to_lowest = (lowest_point_idx - 1) % len(points)
    above_left = (left_to_lowest - 1) % len(points)

    offset = orderred_points[above_left] - orderred_points[left_to_lowest]
    missing_point = orderred_points[lowest_point_idx] + offset

    plt.figure()
    plt.scatter(orderred_points[:, 0], orderred_points[:, 1])
    plt.scatter(missing_point[0], missing_point[1])
    plt.xlim([-1, 5])
    plt.ylim([-1, 4])
    plt.show()

    top_four = np.array([missing_point, orderred_points[above_left], orderred_points[(above_left+1)%len(points)], orderred_points[(above_left+2)%len(points)]])

    return top_four

filename = './images/block.jpg'

img = cv2.imread(filename)
cv2.imshow('Original image', img)
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
gray = np.float32(gray)
dst = cv2.cornerHarris(gray, 2, 3, 0.0038)
corners = find_centroids(dst)
corners = np.delete(corners,0, axis=0)
# To draw the corners
for corner in corners:
   cv2.circle(img,(int(corner[0]), int(corner[1])), 5, (255,0,0), -1)
   #img[int(corner[1]), int(corner[0])] = [0, 0, 255]
cv2.imshow('Found Corners', img)


points = corners
top = up(points)
print(top)
for point in top:
   cv2.circle(img,(int(point[0]), int(point[1])), 5, (0,0,255), -1)

top_rs = top.reshape((-1, 1, 2)).astype(int)
cv2.fillPoly(img, [top_rs], (0, 255, 0))

cv2.imshow('Predicted top', img)
cv2.waitKey(0)
cv2.destroyAllWindows()


