import base64
import cv2 as cv
import numpy as np
import paho.mqtt.client as mqtt
from noPlateRecognition import ANPR
from utils import label_map_util
from utils import visualization_utils as vis_util
import os
import tensorflow as tf
from PIL import Image
import imutils
import pytesseract
import sqlite3 as sql

pytesseract.pytesseract.tesseract_cmd = r'C:\Program Files\Tesseract-OCR\tesseract.exe'
amazon = "18.212.25.214"
pi = "192.168.8.196"
MQTT_BROKER =  pi
MQTT_RECEIVE = "home/photo"

class photoMQTT:
        # The callback for when the client receives a CONNACK response from the server.
        def __init__(self):
            global frame
            self.frame = np.zeros((240, 320, 3), np.uint8)
        
        def on_connect(client, userdata, flags, rc):
            #print("Connected with result code "+str(rc))

            # Subscribing in on_connect() means that if we lose the connection and
            # reconnect then subscriptions will be renewed.
            client.subscribe(MQTT_RECEIVE)


        # The callback for when a PUBLISH message is received from the server.
        def on_message(client, userdata, msg):
            print(msg)
            f = open('output.jpg', "wb")
            f.write(msg.payload)
            print("Image Received")
            f.close()
           
        client = mqtt.Client("photo")
        client.connect(MQTT_BROKER,1883)
        client.subscribe(MQTT_RECEIVE)
        client.on_connect = on_connect
        client.on_message = on_message

        # Starting thread which will receive the frames
        client.loop_start()
        
        def get_photo():
            con = sql.connect("database.db")
            con.row_factory = sql.Row
   
            cur = con.cursor()
            cur.execute("select * from vehicles")
   
            rows = cur.fetchall();
            con.close()

            global frame
            # create object to automatically recognize number plates
            
            anpr = ANPR(debug=False)
            frame = cv.imread("output.jpg")
            frame = imutils.resize(frame, width=600)
            # apply automatic license plate recognition
            (lpText, lpCnt) = anpr.find_and_ocr(frame, psm=7,clearBorder=False)
            
            # only continue if the license plate was successfully OCR'd
            if lpText is not None and lpCnt is not None:
                for row in rows:
                    if lpText in row[0]:
                        print("Suspicious Vehicle Detected")
                    
                # fit a rotated bounding box to the license plate contour and
                # draw the bounding box on the license plate
                box = cv.boxPoints(cv.minAreaRect(lpCnt))
                box = box.astype("int")
                cv.drawContours(frame, [box], -1, (0, 255, 0), 2)

                # compute a normal (unrotated) bounding box for the license
                # plate and then draw the OCR'd license plate text on the
                # image
                (x, y, w, h) = cv.boundingRect(lpCnt)
                cv.putText(frame, anpr.cleanup_text(lpText), (x, y - 15),
                cv.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2)
                # show the output ANPR image
                #print("[INFO] {}".format(lpText))
                #cv.imshow("Output ANPR", frame)
                #cv.waitKey(0)
            ret, jpeg = cv.imencode('.jpg',frame)
            return jpeg.tobytes()
            
        
# if __name__=="__main__":
#     photo = photoMQTT()
#     #photo.get_photo()
#     photo.on_message