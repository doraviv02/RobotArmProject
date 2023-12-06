import numpy as np
import cv2


img = cv2.imread("camera_output.jpg")
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
bilateral = cv2.bilateralFilter(gray, 5, 75, 75)

cv2.imshow("bilateral",bilateral)

dst = cv2.cornerHarris(bilateral, 3, 3, 0.04)
#--- create a black image to see where those corners occur ---
mask = np.zeros_like(gray)

#--- applying a threshold and turning those pixels above the threshold to white ---
mask[dst>0.01*dst.max()] = 255
cv2.imshow('mask', mask)

cv2.waitKey(0)
cv2.destroyAllWindows()