from __future__ import print_function
import os
import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import tensorflow as tf
import argparse
import sys
import flask
import struct

# This is needed since the working directory is the object_detection folder.
sys.path.append('..')

# Import utilites
from utils import label_map_util
from utils import visualization_utils as vis_util

#Libraries
import RPi.GPIO as GPIO
import time, threading

# using socket and pickle for sending video
import socket
import pickle

#clientsocket=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
#clientsocket.connect(('192.168.8.138',8089)) #Change the ip to the desktop ip to check

# Using MQTT
import paho.mqtt.client as mqtt
import base64

# Raspberry PI IP address
#MQTT_BROKER = "192.168.8.196"
# Topic on which frame will be published
#MQTT_SEND = "home/server"

# Phao-MQTT Clinet
#client = mqtt.Client()
# Establishing Connection with the Broker
#client.connect(MQTT_BROKER)

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
        
if __name__ == '__main__':
    AutoLight = threading.Thread(target=AutoLight,daemon=True)
    AutoLight.start()