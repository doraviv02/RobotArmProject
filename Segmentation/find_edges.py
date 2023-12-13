import cv2
import numpy as np
import math


# used for distance approximation, y distance from center
def find_y_diff(center_x, center_y, m, b):
    y_line = m * center_x + b
    return (y_line - center_y)

def pix2cam(center, M):
    center_in_robotsys = np.squeeze(cv2.perspectiveTransform(np.array(center, dtype=np.float32).reshape(-1, 1, 2), M))
    center_in_camsys = -np.array([center_in_robotsys[0] - 610, center_in_robotsys[1]])  # in mm
    return center_in_camsys

def slope_and_intercept(x1, y1, x2, y2):
    m = (y2 - y1) / (x2 - x1)
    b = y1 - m * x1
    return float(m), float(b)


def extract_parameters(x1, y1, x2, y2):
    if (x1 == x2):
        return float(y1), float(0), float(1e10), float(0)
    elif (y1 == y2):
        return float(x1), float(math.pi / 2), float(0), float(y1)
    else:
        # theta = float(math.atan2(y2 - y1, x2-x1))
        m, b = slope_and_intercept(x1, y1, x2, y2)
        x = -m * b / (1 + m ** 2)
        y = b / (1 + m ** 2)
        r = np.abs(np.sqrt(x ** 2 + y ** 2))
        theta = float(np.arctan(y / x))
        # return in degrees
        theta = theta * 180 / np.pi
        return r, theta, m, b


def merge_similar_lines(parameters, lines):
    i, j = 0, 0
    merged_lines = []
    merged_parameters = []
    # radial direction
    while (i < len(lines)):
        j = i + 1
        similar_parameters = [parameters[i].tolist()]
        similar_lines = [lines[i].tolist()]
        # searching in radial direction
        while j < len(lines) and np.abs(parameters[j - 1][1] - parameters[j][1]) < 20:
            similar_parameters.append(parameters[j].tolist())
            similar_lines.append(lines[j].tolist())
            j += 1
        i = j
        merged_parameters.append(similar_parameters)
        merged_lines.append(similar_lines)
    # search wrap around angles, for "almost vertical" lines
    if (np.abs(merged_parameters[0][0][1] + 180 - merged_parameters[-1][-1][1]) < 10):
        merged_parameters[0] += merged_parameters[-1]
        merged_lines[0] += merged_lines[-1]
        merged_parameters.pop(-1)
        merged_lines.pop(-1)

    # create average line m, b
    for idx in range(len(merged_parameters)):
        merged_parameters[idx] = np.average(np.array(merged_parameters[idx]), axis=0)
        merged_lines[idx] = np.average(np.array(merged_lines[idx]), axis=0)

    return merged_parameters


def classify_box(contour, centroid, M):
    linesP = cv2.HoughLinesP(contour, 1, np.pi / 180, 30, None, 30, 10)
    parameters = np.array([extract_parameters(*linesP.squeeze(axis=1)[i]) for i in range(len(linesP))])

    centroid_camera = pix2cam(centroid, M)
    centroid_angle = np.arctan(centroid_camera[1]/centroid_camera[0])*180/np.pi

    # Remove verticals, find average vertical vector
    vertical_parameters = np.array([])
    i = 0
    while (i < len(linesP)):
        #angle= 90-np.arctan(-parameters[i][2])
        #if (np.abs(parameters[i][1]) < 20 or np.abs(parameters[i][1] - 180) < 20):
        if (np.abs(-parameters[i][1]-centroid_angle) < 20 or np.abs(-parameters[i][1] - 180-centroid_angle) < 20):
            vertical_parameters = np.append(vertical_parameters, parameters[i])
            parameters = np.delete(parameters, i, 0)
            linesP = np.delete(linesP, i, 0)
        else:
            i += 1

    # find average vertical parameters
    vertical_parameters = np.average(vertical_parameters.reshape(-1, 4), axis=0)

    linesP = linesP[parameters[:, 1].argsort()]
    parameters = parameters[parameters[:, 1].argsort()]

    top_linesP = []
    top_parameters = []

    for idx in range(len(parameters) - 1, -1, -1):
        if (find_y_diff(*centroid, *parameters[idx][2:]) < 0):
            top_parameters.append(parameters[idx])
            top_linesP.append(linesP[idx][0])

            parameters = np.delete(parameters, idx, 0)
            linesP = np.delete(linesP, idx, 0)

    # flip so lists are in increasing order
    top_parameters = np.array(np.flip(np.array(top_parameters), axis=0))
    top_linesP = np.array(np.flip(np.array(top_linesP), axis=0))

    # blank = np.zeros_like(contour)
    # for l in top_linesP:
    #     cv2.line(blank, (l[0], l[1]), (l[2], l[3]), (255, 255, 255), 3, cv2.LINE_AA)
    # for l in linesP:
    #     cv2.line(blank, (l[0][0], l[0][1]), (l[0][2], l[0][3]), (255, 255, 255), 3, cv2.LINE_AA)
    #
    # cv2.imshow("top", blank)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    merged_parameters_bottom = merge_similar_lines(parameters, linesP)
    merged_parameters_top = merge_similar_lines(top_parameters, top_linesP)


    # TODO: possibly calculate top section based on the bottom and vertical lines.

    if (len(merged_parameters_bottom) == 1):  # block in front of camera
        return 1, (2*merged_parameters_bottom[0] + 2 * merged_parameters_top[0]) / 4
    elif (len(merged_parameters_bottom) == 2):  # block angled from camera
        return 2, None
    else:
        print("Warning: imperfect classification")
        return 2, None
