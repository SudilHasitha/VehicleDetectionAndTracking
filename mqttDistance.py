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
    
            print(distance,"cm")
    except:
        GPIO.cleanup()

if __name__ == '__main__':
    distanceThread = threading.Thread(target=distance,daemon=True)
    distanceThread.start()
    