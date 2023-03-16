import cv2
import qrcode
import numpy as np
from pyzbar.pyzbar import decode


#creates qr code, given data
def create_qr(color):
    #data to encode
    data = color
    
    #creating an instance of QRCode class
    qr = qrcode.QRCode(version = 1,
                    box_size = 10,
                    border = 5)
    
    qr.add_data(data) #encode whatever color is passed in as the data
    qr.make(fit = True)

    #change color of qr codes for visuals
    if color=="W":
        img = qr.make_image(fill_color = "black", back_color = 'white')
    elif color=="R":
        img = qr.make_image(fill_color = "black", back_color = '#FFC8C5')
    elif color=="G":
        img = qr.make_image(fill_color = "black", back_color = '#C1F4B0')
    elif color=="Y":
        img = qr.make_image(fill_color = "black", back_color = '#FFF49C')
    elif color=="B":
        img = qr.make_image(fill_color = "black", back_color = '#CDF3F3')
    elif color=="O":
        img = qr.make_image(fill_color = "black", back_color = '#FFE1AE')
    
    # save it to a file img/<color>_qr.png
    img.save("img/"+color+"_qr.png")



#given an image, detects and decodes multiple qr codes using pyzbar's decode()
def detect_multiple_qr(img):
    data = decode(img)

    for d in data:
        s = d.data.decode()
        
        #bounds the qr code with a green polygon w thickness = 2
        img = cv2.polylines(img, [np.array(d.polygon)], True, (0, 255, 0), 2)

        #superimpose decoded data at bottom left corner of detected qr code
        img = cv2.putText(img, d.data.decode(), (d.rect.left, d.rect.top + d.rect.height+10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)

    cv2.imwrite('img/zbar_output.png', img) #save result in zbar_output.png img

    return data #returns list of decoded data



detect_multiple_qr(cv2.imread("img/merged_image.png"))