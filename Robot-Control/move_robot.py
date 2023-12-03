from xarm.wrapper import XArmAPI
#arm = XArmAPI('COM5')
arm = XArmAPI('192.168.1.165')
arm.connect()

id2_br_x = -4.7 #offset in x from bottom right of id1
id2_br_y = 26 #offset in y from bottom right of id1

centroids = [[173.75473441, 248.46743649],
 [311.67713601, 259.87490019]]

# pix_x = 500-311.90428769
# pix_y = 400-138.95739972

pix_x = 173.75473441
pix_y = 248.46743649

pix_x-=50 #error

width = 52
height = 26

pixel_width = 520
pixel_height = 260

loc_x = pix_x/pixel_width * width + id2_br_x
loc_y = pix_y / pixel_height * height + id2_br_y

arm.clean_error()
arm.set_position_aa([loc_y*10 + 100,loc_x*10,100,180,0,0])

print("x in space:", loc_x)
print("y in space:", loc_y)



