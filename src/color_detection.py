import cv2
import qrcode
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from PIL import Image
from pyzbar.pyzbar import decode

from qr import *

#helper func: checks if data point d is inbetween values n1 and n2
def inbetween(d, n1, n2):
    if d <= n2 and d >= n1:
        return True
    return False

#represents a 3x3 rubiks cube face as string
def get_rubiks_face_grid(data):
    c00 = c01 = c02 = c11 = c12 = c20 = c21 = c22 = data[0].rect[0:2]
    color00 = color01 = color02 = color10 = color11 = color12 = color20 = color21 = color22 = ""
    
    #find qr code locations based on grid
    for i in range(len(data)):
        if inbetween(data[i].rect[0], 400, 600) and inbetween(data[i].rect[1], 60, 260):
            c00 = data[i].rect[0:2]
            color00 = data[i].data

        if inbetween(data[i].rect[0], 600, 800) and inbetween(data[i].rect[1], 60, 260):
            c01 = data[i].rect[0:2]
            color01 = data[i].data

        if inbetween(data[i].rect[0], 800, 1000) and inbetween(data[i].rect[1], 60, 260):
            c02 = data[i].rect[0:2]
            color02 = data[i].data

        if inbetween(data[i].rect[0], 400, 600) and inbetween(data[i].rect[1], 260, 460):
            c10 = data[i].rect[0:2]
            color10 = data[i].data

        if inbetween(data[i].rect[0], 600, 800) and inbetween(data[i].rect[1], 260, 460):
            c11 = data[i].rect[0:2]
            color11 = data[i].data

        if inbetween(data[i].rect[0], 800, 1000) and inbetween(data[i].rect[1], 260, 460):
            c12 = data[i].rect[0:2]
            color12 = data[i].data

        if inbetween(data[i].rect[0], 400, 600) and inbetween(data[i].rect[1], 460, 660):
            c20 = data[i].rect[0:2]
            color20 = data[i].data

        if inbetween(data[i].rect[0], 600, 800) and inbetween(data[i].rect[1], 460, 660):
            c21 = data[i].rect[0:2]
            color21 = data[i].data

        if inbetween(data[i].rect[0], 800, 1000) and inbetween(data[i].rect[1], 460, 660):
            c22 = data[i].rect[0:2]
            color22 = data[i].data

    
    print(color00.decode(), color01.decode(), color02.decode())
    print(color10.decode(), color11.decode(), color12.decode())
    print(color20.decode(), color21.decode(), color22.decode())

    face_colors = color00 + color01 + color02 + color10 + color11 + color12 + color20 + color21 + color22
    face_colors = face_colors.decode() #convert from byte type to string


    #return face colors reading row by row, left to right
    return face_colors #input for kociemba algorithm (once u convert to UDRFLB stuff)

    

#detects multiple qr codes from video
def start_video():
    camera_id = 0
    delay = 1
    window_name = 'OpenCV pyzbar'

    cap = cv2.VideoCapture(camera_id) #define video capture obj

    while True:
        ret, frame = cap.read() #capture each frame

        if ret:
            #draws 3x3 grid template on frame to help align qr codes
            #(img, start, end, color, thickness)
            cv2.line(frame, (400,60), (1000,60), (0,0,255), 9)
            cv2.line(frame, (400,60), (400,660), (0,0,255), 9)
            cv2.line(frame, (400,660), (1000,660), (0,0,255), 9)
            cv2.line(frame, (1000,60), (1000,660), (0,0,255), 9)
            cv2.line(frame, (400,260), (1000,260), (0,0,255), 9)
            cv2.line(frame, (400,460), (1000,460), (0,0,255), 9)
            cv2.line(frame, (600,60), (600,660), (0,0,255), 9)
            cv2.line(frame, (800,60), (800,660), (0,0,255), 9)
            
            decodedObjects = decode(frame) #use pyzbar decode to find multiple qr codes
            if len(decodedObjects) == 9:
                #once a frame finds all 9 qr codes successfully, save the image and break the while loop
                cv2.imwrite("img/cam_ss.png", frame)
                # data_ss = detect_multiple_qr(cv2.imread('img/cam_ss.png'))
                print(get_rubiks_face_grid(decodedObjects))
                
                break

            for d in decodedObjects:
                s = d.data.decode()
                # print(s)
                
                #bound a green rectangle w/thicknesss = 3 around detected qr codes in the frame
                frame = cv2.rectangle(frame, (d.rect.left, d.rect.top),
                                    (d.rect.left + d.rect.width, d.rect.top + d.rect.height), (0, 255, 0), 3)
                #superimpose detected qr code data in red text
                frame = cv2.putText(frame, s, (d.rect.left, d.rect.top + d.rect.height),
                                    cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 2, cv2.LINE_AA)
                
            cv2.imshow(window_name, frame) #display each frame

            
        #to stop video, press 'q' key
        if cv2.waitKey(delay) & 0xFF == ord('q'):
            break


    cap.release()
    cv2.destroyWindow(window_name)


start_video()