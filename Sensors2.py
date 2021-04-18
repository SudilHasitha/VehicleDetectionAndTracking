from __future__ import print_function
import os
import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import sys


# This is needed since the working directory is the object_detection folder.
sys.path.append('..')


#Libraries
import RPi.GPIO as GPIO
import time, threading

# use multi processing instead of threading
from multiprocessing import Process

#clientsocket=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
#clientsocket.connect(('192.168.8.138',8089)) #Change the ip to the desktop ip to check

# Using MQTT
import paho.mqtt.client as mqtt
import base64

def on_connect(client, userdata, flags, rc):
   if rc==0:
      print("connected ok")


# Raspberry PI IP address
pi = "192.168.8.196"
amazon = "18.234.78.84"
MQTT_BROKER = pi

# Topic on which frame will be published
MQTT_SEND = "home/server"
MQTT_SEND_ULTRASONIC = "home/distance"
MQTT_SEND_LIGHT_INTENSITY = "home/light"
MQTT_MANUER = "home/adjust"

# Phao-MQTT Clinet
client = mqtt.Client("raspberry")
# Establishing Connection with the Broker,port number and keep alive
client.connect(MQTT_BROKER,1883,60)

client.on_connect=on_connect  #bind call back function

def distance():   
    try:
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
    
        TRIG = 23
        ECHO = 24
        maxTime = 0.04

        while True:
            GPIO.setup(TRIG,GPIO.OUT)
            GPIO.setup(ECHO,GPIO.IN)
    
            GPIO.output(TRIG,False)
    
            time.sleep(1)
    
            GPIO.output(TRIG,True)
    
            time.sleep(0.001)
    
            GPIO.output(TRIG,False)
    
            pulse_start = time.time()
            timeout = pulse_start + maxTime
            while GPIO.input(ECHO) == 0 and pulse_start < timeout:
                pulse_start = time.time()
    
            pulse_end = time.time()
            timeout = pulse_end + maxTime
            while GPIO.input(ECHO) == 1 and pulse_end < timeout:
                pulse_end = time.time()
    
            pulse_duration = pulse_end - pulse_start
            distance = pulse_duration * 17000
            distance = round(distance, 2)
            client.publish(MQTT_SEND_ULTRASONIC, distance)
            print(distance,"cm")
    except:
        GPIO.cleanup()

def AutoLight():
    try:    
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        mpin=27
        tpin=22
        
        cap=0.00001
        adj=2.130620985
        i=0
        t=0
        while True:
            GPIO.setup(mpin, GPIO.OUT)
            GPIO.setup(tpin, GPIO.OUT)
            GPIO.output(mpin, False)
            GPIO.output(tpin, False)
            time.sleep(0.2)
            GPIO.setup(mpin, GPIO.IN)
            time.sleep(0.2)
            GPIO.output(tpin, True)
            starttime=time.time()
            endtime=time.time()
            while (GPIO.input(mpin) == GPIO.LOW):
                endtime=time.time()
            measureresistance=endtime-starttime
            
            res=(measureresistance/cap)*adj
            client.publish(MQTT_SEND_LIGHT_INTENSITY, res)
            print(res)
            i=i+1
            t=t+res
            if i==10:
                    t=t/i
                    print(t)
                    i=0
                    t=0
    except:
        GPIO.cleanup()

#position servos 
def positionServo (servo, angle):
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(servo, GPIO.OUT)
    pwm = GPIO.PWM(servo, 50)
    pwm.start(7.5)
    dutyCycle = angle / 18. + 2.5 # 90/18 + 2.5 = 7.5
    pwm.ChangeDutyCycle(dutyCycle) # 7.5 -> 0 degree
    time.sleep(0.1)
    pwm.stop()
    GPIO.cleanup()
    
# position servos to present object at center of the frame
def mapServoPosition (x,y):
    global panAngle
    #global tiltAngle need another servo for vertical adjustment
    if (x < 240): # x is the box width
        panAngle += 10
        if panAngle > 180:
           panAngle = 180
        positionServo (panServo, panAngle)

    if (x > 260):
        panAngle -= 10
        if panAngle < 0 :
            panAngle = 0
        positionServo (panServo, panAngle)

def ObjectTrackingCamera():
    
    # remove gpio warnings and set board numbering method
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    
    # define sensor pins
    global panServo 
    panServo = 17
    
    # intialize the motor angle
    global panAngle
    panAngle = 80
    mapServoPosition(panServo,panAngle)

    while True:
        # Initialize Picamera and grab reference to the raw capture
        camera = PiCamera()
        sWidth = 640
        sHeight = 480
        camera.resolution = (sWidth,sHeight)
        camera.framerate = 90
        rawCapture = PiRGBArray(camera, size=(sWidth,sHeight))
        rawCapture.truncate(0)

        for frame in camera.capture_continuous(rawCapture, format="bgr",use_video_port=True):
            frame = np.copy(frame.array)
            frame.setflags(write=1)
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frame_expanded = np.expand_dims(frame_rgb, axis=0)
            # Encoding the Frame
            _, buffer = cv2.imencode('.jpg', frame)
            # Converting into encoded bytes
            jpg_as_text = base64.b64encode(buffer)
            # Publishig the Frame on the Topic home/server
            client.publish(MQTT_SEND, jpg_as_text)
            
            # Press 'q' to quit
            if cv2.waitKey(1) == ord('s'):
                break

            rawCapture.truncate(0)
            
        panAngle = 80
        camera.close()
        GPIO.cleanup()



if __name__ == '__main__':
    
    ObjectTrackingProcess = Process(target=ObjectTrackingCamera)
    ObjectTrackingProcess.start()
    
    DistanceProcess = Process(target=distance)
    DistanceProcess.start()
    
    AutoLightProcess = Process(target=AutoLight)
    AutoLightProcess.start()
    
    # Listen for q key
    k = cv2.waitKey(1) & 0xFF
    if k == ord('q'):
        ObjectTrackingProcess.terminate()
        DistanceProcess.terminate()
        AutoLightProcess.terminate()
