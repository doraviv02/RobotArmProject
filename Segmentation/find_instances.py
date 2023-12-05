import cv2
import numpy as np

def run(original_image_path, mask_image_path):
    original_image = cv2.imread(original_image_path)
    # Load the image in grayscale
    mask_image = cv2.imread(mask_image_path, cv2.IMREAD_GRAYSCALE)


    # Threshold your image to make sure that is binary
    # thresh_type = cv2.THRESH_BINARY + cv2.THRESH_OTSU
    # _, binary_image = cv2.threshold(mask_image, 0, 255, thresh_type)
    thresh=125

    mask_image[mask_image>=thresh] = 255
    mask_image[mask_image<thresh] = 0
    binary_image=mask_image


    # Perform connected component labeling
    n_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(binary_image,
                                                                          connectivity=4)
    # Create false color image
    colors = np.random.randint(0, 255, size=(n_labels , 3), dtype=np.uint8)
    colors[0] = [0, 0, 0]  # for cosmetic reason we want the background black
    false_colors = colors[labels]

    # Perform close operation on binary_image
    binary_image = cv2.morphologyEx(binary_image, cv2.MORPH_CLOSE, np.ones((10, 10), np.uint8))
    binary_image = cv2.morphologyEx(binary_image, cv2.MORPH_RECT, np.ones((3, 3), np.uint8))

    MIN_AREA = 50
    false_colors_draw = false_colors.copy()

    segmented = []

    # Loop over the centroids and labels
    for i, centroid in enumerate(centroids[1:], start=1):
        area = stats[i, 4]
        if area > MIN_AREA:
            #new_image = original_image.copy()
            mask = ((labels==i)*255).astype(np.uint8)
            #new_image = cv2.bitwise_and(new_image,new_image, mask= mask)
            #cv2.imshow("filtered " +str(i) ,new_image)
            segmented.append(mask)

            #print(new_image[new_image[:,:,0]!=0][:,0])
            #avg_red = np.average(new_image[new_image[:,:,1]!=0][:,1])
            #print("average green: "+str(avg_red))
            #cv2.drawMarker(false_colors_draw, (int(centroid[0]), int(centroid[1])),
                           #color=(255, 255, 255), markerType=cv2.MARKER_CROSS)

    #cv2.waitKey(0)

    return segmented
