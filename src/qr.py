import cv2
import qrcode
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from PIL import Image
from pyzbar.pyzbar import decode


#creates qr code, given data
def create_qr(color):
    #data to encode
    data = color
    
    #creating an instance of QRCode class
    qr = qrcode.QRCode(version = 1,
                    box_size = 10,
                    border = 5)
    
    qr.add_data(data)
    
    qr.make(fit = True)

    #change color of qr codes for visuals
    if color=="W":
        img = qr.make_image(fill_color = "black", back_color = 'white')
    elif color=="R":
        img = qr.make_image(fill_color = "red", back_color = 'white')
    elif color=="G":
        img = qr.make_image(fill_color = "green", back_color = 'white')
    elif color=="Y":
        img = qr.make_image(fill_color = "darkkhaki", back_color = 'white')
    elif color=="B":
        img = qr.make_image(fill_color = "navy", back_color = 'white')
    elif color=="O":
        img = qr.make_image(fill_color = "darkorange", back_color = 'white')
    
    # save it to a file img/<color>_qr.png
    img.save("img/"+color+"_qr.png")



#given an image, detects and decodes multiple qr codes using pyzbar's decode()
def detect_multiple_qr(img):
    data = decode(img)

    for d in data:
        s = d.data.decode()
        # print(s)
        
        #cv2.rectangle(img, start_rect, end_rect, BGR color, thickness)
        #bounds the qr code with a blue rectangle w thickness = 2
        img = cv2.rectangle(img, (d.rect.left, d.rect.top),
                            (d.rect.left + d.rect.width, d.rect.top + d.rect.height), (255, 0, 0), 2)
        #bounds the qr code with a green polygon w thickness = 2
        img = cv2.polylines(img, [np.array(d.polygon)], True, (0, 255, 0), 2)

        #superimpose decoded data at bottom left corner of detected qr code
        img = cv2.putText(img, d.data.decode(), (d.rect.left, d.rect.top + d.rect.height+10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)

    cv2.imwrite('img/zbar_output.png', img)

    return data #returns list of decoded data

