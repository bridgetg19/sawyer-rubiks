import cv2
import numpy as np
import matplotlib.pyplot as plt
import kociemba

from color_detect import *
from kociemba_stuff import *

# --- global variables -------------------------------------------------------
x1, x2 = 325, 955 #starting & ending x coordinate of grid template
y1, y2 = 45, 675 #starting & ending y coordinate of grid template
cell_size = 210 #size of cell from grid template
cx = 640 #center x coordinate of frame
cy = 360 #center y coordinate of frame


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
    color=(0,0,255)
    thickness = 5

    cv2.line(frame, (x1,y1), (x2,y1), color, thickness) #top
    cv2.line(frame, (x1,y1), (x1,y2), color, thickness) #left
    cv2.line(frame, (x1,y2), (x2,y2), color, thickness) #bottom
    cv2.line(frame, (x2,y1), (x2,y2), color, thickness) #right

    cv2.line(frame, (x1, y1+cell_size), (x2, y1+cell_size), color, thickness) #horizontal 1
    cv2.line(frame, (x1, y1+(cell_size*2)), (x2, y1+(cell_size*2)), color, thickness) #horizontal 2
    cv2.line(frame, (x1+cell_size, y1), (x1+cell_size, y2), color, thickness) #vertical 1
    cv2.line(frame, (x1+(cell_size*2), y1), (x1+(cell_size*2), y2), color, thickness) #vertical 2

    height, width, _ = frame.shape
    cx = width // 2
    cy = height // 2

    # cv2.circle(image, center_coordinates, radius, color, thickness)
    cv2.circle(frame, (cx, cy), 10, color, thickness)
    cv2.circle(frame, (cx + cell_size, cy), 10, color, thickness)
    cv2.circle(frame, (cx - cell_size, cy), 10, color, thickness)
    cv2.circle(frame, (cx, cy + cell_size), 10, color, thickness)
    cv2.circle(frame, (cx + cell_size, cy + cell_size), 10, color, thickness)
    cv2.circle(frame, (cx - cell_size, cy + cell_size), 10, color, thickness)
    cv2.circle(frame, (cx, cy - cell_size), 10, color, thickness)
    cv2.circle(frame, (cx + cell_size, cy - cell_size), 10, color, thickness)
    cv2.circle(frame, (cx - cell_size, cy - cell_size), 10, color, thickness)



# starts video to take image of each face of the cube (use 's' key input to save image)
def start_video():
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    face_count  = 0

    while True:
        _, frame = cap.read()
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        cell_centers = get_cell_centers()

        draw_template(frame)
        
        cv2.imshow('frame', frame)

        #take ss and save image in "img/face_x.png"
        key = cv2.waitKey(1)
        if key == ord('s'):
            img_name = "img/face_" + str(face_count + 1) + ".png"
            cv2.imwrite(img_name, frame)
            print("saving file", img_name)

            face_count += 1


        if face_count == 6:
            print("read all 6 sides, exiting")
            break

        #quit out
        key = cv2.waitKey(1) 
        if key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

    return cell_centers


def main():
    #start video to get all sides
    start_video()

    #take images from the video and determine the colors
    cell_centers = get_cell_centers()
    f1, h1 = color_detect(cv2.imread('img/face_1.png'), cell_centers)
    f2, h2 = color_detect(cv2.imread('img/face_2.png'), cell_centers)
    f3, h3 = color_detect(cv2.imread('img/face_3.png'), cell_centers)
    f4, h4 = color_detect(cv2.imread('img/face_4.png'), cell_centers)
    f5, h5 = color_detect(cv2.imread('img/face_5.png'), cell_centers)
    f6, h6 = color_detect(cv2.imread('img/face_6.png'), cell_centers)

    #replace center tile of face 1 with "W" bc of logo
    f1[4] = 'W'

    #concatenate all the faces together
    rubiks_cube = list_to_string(f1) + list_to_string(f2) + list_to_string(f3) + list_to_string(f4) + list_to_string(f5) + list_to_string(f6)
    
    #translate string from WYROGB to UDRLFB
    kociemba_input = translate_string(rubiks_cube)

    #send to kociemba algorithm
    print(kociemba.solve(kociemba_input))



if __name__ == "__main__":
    main()