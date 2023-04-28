import cv2
import numpy as np
import matplotlib.pyplot as plt
import kociemba
import time

from color_detect import *
from kociemba_stuff import *

# --- global variables -------------------------------------------------------
x1, x2 = 425, 900 #starting & ending x coordinate of grid template
y1, y2 = 150, 600 #starting & ending y coordinate of grid template
cell_size = 158 #size of cell from grid template
cx = 662 #center x coordinate of frame
cy = 382 #center y coordinate of frame


# gets the center pixel coordinate of each cell in the 3x3 grid templae
def get_cell_centers():
    cell_centers=[]

    cell_centers.append((cx - cell_size, cy - cell_size)) #top left
    cell_centers.append((cx, cy - cell_size)) #top center
    cell_centers.append((cx + cell_size, cy - cell_size)) #top right
    cell_centers.append((cx - cell_size, cy)) #left middle
    cell_centers.append((cx, cy)) #center
    cell_centers.append((cx + cell_size, cy)) #right middle
    cell_centers.append((cx - cell_size, cy + cell_size)) #bottom left
    cell_centers.append((cx, cy + cell_size)) #bottomn center
    cell_centers.append((cx + cell_size, cy + cell_size)) #bottom right

    return cell_centers



# draws the 3x3 grid template on frame in video
def draw_template(frame):
    thickness = 5
    color = (0,255,0)

    # cv2.circle(image, center_coordinates, radius, color, thickness)
    #top row
    cv2.circle(frame, (cx - cell_size - 30, cy - cell_size - 30), 10, color, thickness)
    cv2.circle(frame, (cx, cy - cell_size - 30), 10, color, thickness)
    cv2.circle(frame, (cx + cell_size + 30, cy - cell_size - 30), 10, color, thickness)

    #middle row
    cv2.circle(frame, (cx - cell_size - 30, cy), 10, color, thickness)
    cv2.circle(frame, (cx, cy), 10, color, thickness)
    cv2.circle(frame, (cx + cell_size + 30, cy), 10, color, thickness)

    #bottom row
    cv2.circle(frame, (cx - cell_size - 30, cy + cell_size + 30), 10, color, thickness)
    cv2.circle(frame, (cx, cy + cell_size + 30), 10, color, thickness)
    cv2.circle(frame, (cx + cell_size + 30, cy + cell_size + 30), 10, color, thickness)


#prints out flattened out version of cube
def chunk_face(f):
    return [f[x:x+3] for x in range(0, len(f), 3)]

def net_cube(f1, f2, f3, f4, f5, f6):
    top = chunk_face(f1)
    print(" "*7, "- "*5)
    print(" "*7, "|", top[0][0], top[0][1], top[0][2], "|")
    print(" "*7, "|", top[1][0], top[1][1], top[1][2], "|")
    print(" "*7, "|", top[2][0], top[2][1], top[2][2], "|")
    print(" "*7, "- "*5)

    left = chunk_face(f5)
    front = chunk_face(f3)
    right = chunk_face(f2)
    back = chunk_face(f6)
    print("- "*17)
    print("|", left[0][0], left[0][1], left[0][2], "|", front[0][0], front[0][1], front[0][2], "|", right[0][0], right[0][1], right[0][2], "|", back[0][0], back[0][1], back[0][2], "|")
    print("|", left[1][0], left[1][1], left[1][2], "|", front[1][0], front[1][1], front[1][2], "|", right[1][0], right[1][1], right[1][2], "|", back[1][0], back[1][1], back[1][2], "|")
    print("|",left[2][0], left[2][1], left[2][2], "|", front[2][0], front[2][1], front[2][2], "|", right[2][0], right[2][1], right[2][2], "|", back[2][0], back[2][1], back[2][2], "|")
    print("- "*17)

    down = chunk_face(f4)
    print(" "*7, "- "*5)
    print(" "*7, "|", down[0][0], down[0][1], down[0][2], "|")
    print(" "*7, "|", down[1][0], down[1][1], down[1][2], "|")
    print(" "*7, "|", down[2][0], down[2][1], down[2][2], "|")
    print(" "*7, "- "*5)


# starts video to take picture of side of rubiks cube
def start_video_single(img_name):
    try:
        cap = cv2.VideoCapture(1)
    except:
        cap = cv2.VideoCapture(0)
    # cap = cv2.VideoCapture(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    while True:
        _, frame = cap.read()
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        cell_centers = get_cell_centers()

        draw_template(frame)
        
        cv2.imshow('frame', frame)

        #take ss and save image in "img/face_x.png"
        time.sleep(3)
        cv2.imwrite(img_name, frame)
        print("saving file", img_name)
        break

        #quit out
        key = cv2.waitKey(1) 
        if key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

