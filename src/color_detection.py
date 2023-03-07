import cv2
import qrcode
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from PIL import Image
from pyzbar.pyzbar import decode

from qr import *

#draws 3x3 grid template on frame in video
def draw_template(frame):
    color=(0,0,255)
    thickness = 5

    x1, x2 = 285, 915
    y1, y2 = 45, 675
    cell_size = 210
    
    cv2.line(frame, (x1,y1), (x2,y1), color, thickness) #top
    cv2.line(frame, (x1,y1), (x1,y2), color, thickness) #left
    cv2.line(frame, (x1,y2), (x2,y2), color, thickness) #bottom
    cv2.line(frame, (x2,y1), (x2,y2), color, thickness) #right

    cv2.line(frame, (x1, y1+cell_size), (x2, y1+cell_size), color, thickness) #horizontal 1
    cv2.line(frame, (x1, y1+(cell_size*2)), (x2, y1+(cell_size*2)), color, thickness) #horizontal 2
    cv2.line(frame, (x1+cell_size, y1), (x1+cell_size, y2), color, thickness) #vertical 1
    cv2.line(frame, (x1+(cell_size*2), y1), (x1+(cell_size*2), y2), color, thickness) #vertical 2



#helper func: checks if data point d is inbetween values n1 and n2
def inbetween(d, n1, n2):
    if d <= n2 and d >= n1:
        return True
    return False



#translates face of rubiks cube into string 
def get_rubiks_face(data):
    x1, x2 = 285, 915
    y1, y2 = 45, 675
    cell_size = 210
    color00 = color01 = color02 = color10 = color11 = color12 = color20 = color21 = color22 = b'' #initialize as empty bytes
    
    #for each decoded qr code in data, find qr code cell position based on top left coordinate of qr code
    for i in range(len(data)):
        if inbetween(data[i].rect[0], x1, x1+cell_size) and inbetween(data[i].rect[1], y1, y1+cell_size):
            color00 = data[i].data

        if inbetween(data[i].rect[0], x1+cell_size, x1+cell_size*2) and inbetween(data[i].rect[1], y1, y1+cell_size):
            color01 = data[i].data

        if inbetween(data[i].rect[0], x1+cell_size*2, x2) and inbetween(data[i].rect[1], y1, y1+cell_size):
            color02 = data[i].data

        if inbetween(data[i].rect[0], x1, x1+cell_size) and inbetween(data[i].rect[1], y1+cell_size, y1+cell_size*2):
            color10 = data[i].data

        if inbetween(data[i].rect[0], x1+cell_size, x1+cell_size*2) and inbetween(data[i].rect[1], y1+cell_size, y1+cell_size*2):
            color11 = data[i].data

        if inbetween(data[i].rect[0], x1+cell_size*2, x2) and inbetween(data[i].rect[1], y1+cell_size, y1+cell_size*2):
            color12 = data[i].data

        if inbetween(data[i].rect[0], x1, x1+cell_size) and inbetween(data[i].rect[1], y1+cell_size*2, y2):
            color20 = data[i].data

        if inbetween(data[i].rect[0], x1+cell_size, x1+cell_size*2) and inbetween(data[i].rect[1], y1+cell_size*2, y2):
            color21 = data[i].data

        if inbetween(data[i].rect[0], x1+cell_size*2, x2) and inbetween(data[i].rect[1], y1+cell_size*2, y2):
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
    window_name = 'Read Rubiks Cube'
    cap = cv2.VideoCapture(camera_id) #define video capture obj

    face_count = 0 #counter for number of frames successfully read


    while True:
        ret, frame = cap.read() #capture each frame

        if ret:
            #draws 3x3 grid template on frame to help align qr codes
            draw_template(frame)

            #show text asking what face to show
            text = "Show face" + str(face_count+1)
            frame = cv2.putText(frame, text, (50,50), cv2.FONT_HERSHEY_SIMPLEX,
                                2, (255, 0, 0), 2, cv2.LINE_AA)
            
                    
            decodedObjects = decode(frame) #use pyzbar decode to find multiple qr codes
            if len(decodedObjects) == 9: #once a frame finds all 9 qr codes successfully, save the image and break the while loop
                #save frame into img file
                img_name = "img/face_" + str(face_count + 1) + ".png"
                cv2.imwrite(img_name, frame)
                print("saving file", img_name)

                frame = cv2.putText(frame, "scanned successfully", (50,50), cv2.FONT_HERSHEY_SIMPLEX,
                                2, (255, 0, 0), 2, cv2.LINE_AA)
                
                face_count += 1 #increment
                
                # cv2.imwrite("img/cam_ss.png", frame)
                # print(get_rubiks_face(decodedObjects))

                # break


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


        #to screenshot frame
        if cv2.waitKey(delay) & 0xFF == ord('s'):
            cv2.imwrite("img/frame_ss.png", frame)
            break

            
        #to stop video, press 'q' key
        if cv2.waitKey(delay) & 0xFF == ord('q'):
            break

        if face_count == 6:
            print("got all sides")
            break


    cap.release()
    cv2.destroyWindow(window_name)




def main():
    start_video()

if __name__ == "__main__":
    main()