import cv2
import numpy as np
import matplotlib.pyplot as plt

# determine color of a tile based on its hsv values
def get_color(hsv):
    hue = hsv[0]
    saturation = hsv[1]

    color = None
    if hue < 5:
        color = "R"
    elif hue < 22:
        color = "O"
    elif hue < 33:
        color = "Y"
    elif hue < 90:
        color = "G"
    elif hue < 131:
        color = "B"
    elif hue < 180:
        color = "R"
    else:
        color = "W"

    if saturation < 75:
        color = "W"

    return color



# determine color state of a rubiks cube face
def color_detect(img, cell_centers):
    cell_colors = []
    hue_values = []
    hsv_frame = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    #find color of each center pixel coordinate of each cell in 3x3 grid
    c00 = hsv_frame[ cell_centers[0][1], cell_centers[0][0]]
    color00 = get_color(c00)

    c01 = hsv_frame[ cell_centers[1][1], cell_centers[1][0]]
    color01 = get_color(c01)

    c02 = hsv_frame[ cell_centers[2][1], cell_centers[2][0]]
    color02 = get_color(c02)

    c10 = hsv_frame[ cell_centers[3][1], cell_centers[3][0]]
    color10 = get_color(c10)

    c11 = hsv_frame[ cell_centers[4][1], cell_centers[4][0]]
    color11 = get_color(c11)

    c12 = hsv_frame[cell_centers[5][1], cell_centers[5][0]]
    color12 = get_color(c12)

    c20 = hsv_frame[ cell_centers[6][1], cell_centers[6][0]]
    color20 = get_color(c20)

    c21 = hsv_frame[ cell_centers[7][1], cell_centers[7][0]]
    color21 = get_color(c21)

    c22 = hsv_frame[ cell_centers[8][1], cell_centers[8][0]]
    color22 = get_color(c22)

    #add all the colors in a list (reading top to bottom, left to right)
    cell_colors.append(color00)
    cell_colors.append(color01)
    cell_colors.append(color02)
    cell_colors.append(color10)
    cell_colors.append(color11)
    cell_colors.append(color12)
    cell_colors.append(color20)
    cell_colors.append(color21)
    cell_colors.append(color22)
    
    hue_values.append(c00)
    hue_values.append(c01)
    hue_values.append(c02)
    hue_values.append(c10)
    hue_values.append(c11)
    hue_values.append(c12)
    hue_values.append(c20)
    hue_values.append(c21)
    hue_values.append(c22)

    return cell_colors, hue_values