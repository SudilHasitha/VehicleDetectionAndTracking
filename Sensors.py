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
MQTT_BROKER = "192.168.8.196"

# Topic on which frame will be published
MQTT_SEND = "home/server"
MQTT_SEND_ULTRASONIC = "home/distance"
MQTT_SEND_LIGHT_INTENSITY = "home/light"

# Phao-MQTT Clinet
client = mqtt.Client()
# Establishing Connection with the Broker
client.connect(MQTT_BROKER)

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
    pwm.start(8)
    dutyCycle = angle / 18. + 3.
    pwm.ChangeDutyCycle(dutyCycle)
    time.sleep(0.1)
    pwm.stop()
    GPIO.cleanup()
    
# position servos to present object at center of the frame
def mapServoPosition (x,y):
    global panAngle
    #global tiltAngle
    if (x < 220):
        panAngle += 10
        if panAngle >= 180:
           panAngle = 180
        positionServo (panServo, panAngle)

    if (x > 280):
        panAngle -= 10
        if panAngle <= 0 :
            panAngle = 0
        positionServo (panServo, panAngle)

def ObjectTrackingCamera():
    
    # remove gpio warnings and set board numbering method
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    
    # define sensor pins
    global panServo 
    panServo = 17
    
    ################### Object detection ###################################
    # Name of the directory containing the object detection module we're using
    MODEL_NAME = 'ssdlite_mobilenet_v2_coco_2018_05_09'

    # Grab path to current working directory
    CWD_PATH = os.getcwd()

    # Path to frozen detection graph .pb file, which contains the model that is used
    # for object detection.
    PATH_TO_CKPT = os.path.join(CWD_PATH,MODEL_NAME,'frozen_inference_graph.pb')

    # Path to label map file
    PATH_TO_LABELS = os.path.join(CWD_PATH,'data','mscoco_label_map.pbtxt')

    # Number of classes the object detector can identify
    NUM_CLASSES = 90
    label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
    categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
    category_index = label_map_util.create_category_index(categories)

    # Load the Tensorflow model into memory.
    detection_graph = tf.Graph()

    with detection_graph.as_default():
        od_graph_def = tf.GraphDef()
        with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
            serialized_graph = fid.read()
            od_graph_def.ParseFromString(serialized_graph)
            tf.import_graph_def(od_graph_def, name='')
        sess = tf.Session(graph=detection_graph)

    image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
    detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
    detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
    detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')
    num_detections = detection_graph.get_tensor_by_name('num_detections:0')

    # Initialize frame rate calculation
    frame_rate_calc = 1
    freq = cv2.getTickFrequency()
    font = cv2.FONT_HERSHEY_SIMPLEX

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
        camera.framerate = 50
        rawCapture = PiRGBArray(camera, size=(sWidth,sHeight))
        rawCapture.truncate(0)

        for frame1 in camera.capture_continuous(rawCapture, format="bgr",use_video_port=True):
            t1 = cv2.getTickCount()
            
            # Acquire frame and expand frame dimensions to have shape: [1, None, None, 3]
            # i.e. a single-column array, where each item in the column has the pixel RGB value

            frame = np.copy(frame1.array)
            frame.setflags(write=1)
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frame_expanded = np.expand_dims(frame_rgb, axis=0)

            # Perform the actual detection by running the model with the image as input
            (boxes, scores, classes, num) = sess.run(
                [detection_boxes, detection_scores, detection_classes, num_detections],
                feed_dict={image_tensor: frame_expanded})
               
            # get the decteded object
            ObjectDetected = [category_index.get(value) for index,value in enumerate(classes[0]) if scores[0,index] > 0.5]
            #print(ObjectDetected)        
            if len(ObjectDetected) != 0:
                if (ObjectDetected[0])["name"] == "person":
                    #print("Person")
            
                    # Detecting person and tracking using the camera
         
                    # Draw the results of the detection (aka 'visulaize the results')        
                    vis_util.visualize_boxes_and_labels_on_image_array(
                        frame,
                        np.squeeze(boxes),
                        np.squeeze(classes).astype(np.int32),
                        np.squeeze(scores),
                        category_index,
                        use_normalized_coordinates=True,
                        line_thickness=8,
                        min_score_thresh=0.40)
                    
                    # get the positions of the detection box
                    box = np.squeeze(boxes)
                    boxPerson = list(box[0])
                    [ymin, xmin, ymax, xmax] = boxPerson
                    
                    # convert to pixcel values
                    left = sWidth * xmin
                    right = sWidth * xmax
                    top = sHeight * ymin
                    bottom = sHeight * ymax
                    
                    # get the center point
                    center = (left + right)/2, (top + bottom)/2
                    #print(center)
                    
                    # get coordinates for the center point
                    x = int(center[0])
                    y = int(center[1])
            
                    # draw the center point
                    cv2.circle(frame,(x,y), 6, (0,0,255), -1)
                    
                    # passing coordinates to adjust the camera
                    mapServoPosition (x,y)
                    
            cv2.putText(frame,"FPS: {0:.2f}".format(frame_rate_calc),(30,50),font,1,(255,255,0),2,cv2.LINE_AA)
            # All the results have been drawn on the frame, so it's time to display it.
            sendFrame = frame
            #cv2.imshow('Object detector', frame)
            t2 = cv2.getTickCount()
            time1 = (t2-t1)/freq
            frame_rate_calc = 1/time1
            
            # Sending a copy of video frame
            #data = pickle.dumps(sendFrame) ### new code
            #clientsocket.sendall(struct.pack("<L", len(data))+data) ### new code
            
            ### MQTT based frame send
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
     
    ObjectTrackingThread = threading.Thread(target=ObjectTrackingCamera,daemon=True)
    ObjectTrackingThread.start()
    
    distanceThread = threading.Thread(target=distance,daemon=True)
    distanceThread.start()
    
    AutoLight = threading.Thread(target=AutoLight,daemon=True)
    AutoLight.start()
    
    