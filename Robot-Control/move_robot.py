from xarm.wrapper import XArmAPI
import numpy as np
import cv2
#arm = XArmAPI('COM5')
arm = XArmAPI('192.168.1.165')
arm.connect()

initial_position = [200,0,200,180,0,0]


# top left, top right, bottom left, bottom right
points = np.float32([[120.6,-250.9],[126, 251.5],[349.4, -255.6],[352.2,251.2]])

#moving robot
#arm.set_position_aa([*points[3],100,180,0,0],wait=True)
#arm.set_position_aa(initial_position,wait=True)

centroids = np.array([[262.67329388, 129.65320405], [123.04614999, 118.61148733], [341.47653745, 124.60886039], [213.34393363, 143.69305219]])
#centroids = np.flip([[262.67329388, 129.65320405], [123.04614999, 118.61148733], [341.47653745, 124.60886039], [213.34393363, 143.69305219]], axis=-1)
projected_size = [260, 520]  # x and y are flipped in the robot
border = np.float32([[0, 0], [520, 0], [0, 260], [520, 260]])

M = cv2.getPerspectiveTransform(border, points)
#print(centroids.reshape(-1,1,2))
print(cv2.perspectiveTransform(centroids.reshape(-1,1,2),M))

arm.set_position_aa([248.19852833, -44.95346764,90,180,0,0],wait=True)

# arm.get_init
# id2_br_x = -4.7 #offset in x from bottom right of id1
# id2_br_y = 26 #offset in y from bottom right of id1
#
# centroids = [[173.75473441, 248.46743649],
#  [311.67713601, 259.87490019]]
#
# # pix_x = 500-311.90428769
# # pix_y = 400-138.95739972
#
# pix_x = 173.75473441
# pix_y = 248.46743649
#
# pix_x-=50 #error
#
# width = 52
# height = 26
#
# pixel_width = 520
# pixel_height = 260
#
# loc_x = pix_x/pixel_width * width + id2_br_x
# loc_y = pix_y / pixel_height * height + id2_br_y
#
# arm.clean_error()
# arm.set_position_aa([loc_y*10 + 100,loc_x*10,100,180,0,0])
#
# print("x in space:", loc_x)
# print("y in space:", loc_y)



