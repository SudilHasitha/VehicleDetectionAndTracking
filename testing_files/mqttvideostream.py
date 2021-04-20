import base64
import cv2 as cv
import numpy as np
import paho.mqtt.client as mqtt

amazon = "18.212.25.214"
pi = "192.168.8.196"
MQTT_BROKER =  pi
MQTT_RECEIVE = "home/server"

class videoStreamMQTT:
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
            global frame
            # Decoding the message
            img = base64.b64decode(msg.payload)
            # converting into numpy array from buffer
            npimg = np.frombuffer(img, dtype=np.uint8)
            # Decode to Original Frame
            frame = cv.imdecode(npimg, 1)
            #frame = cv.imdecode(".jpg",npimg)
            
        
        client = mqtt.Client("video")
        client.connect(MQTT_BROKER,1883)
        client.subscribe(MQTT_RECEIVE)
        client.on_connect = on_connect
        client.on_message = on_message

        # Starting thread which will receive the frames
        client.loop_start()
        
        def get_frame():
            global frame
            ret, jpeg = cv.imencode('.jpg',frame)
            return jpeg.tobytes()
            
        
        

