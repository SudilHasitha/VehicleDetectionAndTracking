from __future__ import print_function
import os
import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import tensorflow as tf
import sys
import copy
from noPlateRecognition import ANPR
from PIL import Image
import statistics
# This is needed since the working directory is the object_detection folder.
sys.path.append('..')

# Import utilites
from utils import label_map_util
from utils import visualization_utils as vis_util

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

from alamSuspicious import alamSuspicious
from gateController import gateController

def on_connect(client, userdata, flags, rc):
   if rc==0:
       pass
      #print("connected ok")


# Raspberry PI IP address
pi = "192.168.8.196"
amazon = "18.234.78.84"
MQTT_BROKER = pi

# Topic on which frame will be published
MQTT_SEND = "home/server"
MQTT_SEND_ULTRASONIC = "home/distance"
MQTT_SEND_LIGHT_INTENSITY = "home/light"
MQTT_LIGHT = "home/LED"
MQTT_BUZZER = "home/buzzer"
MQTT_PHOTO = "home/photo"

# Phao-MQTT Clinet
client = mqtt.Client("raspberry")
# Establishing Connection with the Broker,port number and keep alive
client.connect(MQTT_BROKER,1883,60)

client.on_connect=on_connect  #bind call back function


def LED():
        
        def on_connect(client,userdata,flags,rc):
            print("connected")
            client.subscribe(MQTT_LIGHT)
            
        def on_message(client,userdata,msg):
            global res
            res = msg.payload
            LEDrun(int.from_bytes(res))
            
 
            
        client.connect(MQTT_BROKER,1883)
        client.subscribe(MQTT_LIGHT)
        
        client.on_connect = on_connect
        client.on_message = on_message
        
        client.loop_start()
        client.on_message = on_message
        
        def LEDrun(res):
                if res > 10000:
                        GPIO.setup(13,GPIO.OUT)
                        GPIO.output(13,GPIO.HIGH)
                else:
                        GPIO.output(13,GPIO.LOW)
                
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
            
    except:
        GPIO.cleanup()

def AutoLight():
    global res
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
            global res
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
         
            i=i+1
            t=t+res
            if i==10:
                    t=t/i
                    i=0
                    t=0
            
            client.publish(MQTT_SEND_LIGHT_INTENSITY, round(res,2))
            
            #'''
            print(res)
            if copy.copy(res) > 10000:
                GPIO.setmode(GPIO.BCM)
                GPIO.setup(13,GPIO.OUT)
                GPIO.output(13,GPIO.HIGH)
            else:
                GPIO.output(13,GPIO.LOW)
            #'''
    except:
        GPIO.cleanup()

    
#position servos 
def positionServo (servo, angle):
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(servo, GPIO.OUT)
    pwm = GPIO.PWM(servo, 50)
    pwm.start(8)
    dutyCycle = angle / 18. + 3 # 90/18 + 2.5 = 7.5
    pwm.ChangeDutyCycle(dutyCycle) # 7.5 -> 0 degree
    time.sleep(0.3)
    pwm.stop()
    GPIO.cleanup()
    
# position servos to present object at center of the frame
def mapServoPosition (x,y):
    global panAngle
    #global tiltAngle need another servo for vertical adjustment
    if (x < 220): # x is the box width
        panAngle += 10
        if panAngle > 140:
           panAngle = 140
        positionServo (panServo, panAngle)

    if (x > 280):
        panAngle -= 10
        if panAngle < 40 :
            panAngle = 40
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
        camera.framerate = 20
        rawCapture = PiRGBArray(camera, size=(sWidth,sHeight))
        rawCapture.truncate(0)

        for frame1 in camera.capture_continuous(rawCapture, format="bgr",use_video_port=True):           
            
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
            ObjectDetected = [category_index.get(value) for index,value in enumerate(classes[0]) if scores[0,index] > 0.01]
            
            #print(ObjectDetected)        
            if len(ObjectDetected) != 0:
                if (ObjectDetected[0])["name"] == "car" or (ObjectDetected[0])["name"] == "truck" or (ObjectDetected[0])["name"] == "suv":
                #if (ObjectDetected[0])["name"] == "person":
                    car = True
                    if car:
                        BUZZER = 12
                        GPIO.setmode(GPIO.BCM)
                        GPIO.setup(BUZZER, GPIO.OUT)
                        GPIO.output(BUZZER, True)
                        time.sleep(0.5)
                        GPIO.output(BUZZER,False)
                        car = False
                    
                    results = ""
                    try:
                        for i in range(1):
                            # get the vehicle number
                            frameNoRec = copy.deepcopy(frame)
                            cv2.imwrite(str(i)+'.jpg',frameNoRec)
                        
                        for i in range(1):
                            # get the vehicle number
                            #image = Image.open("001.jpg")
                            #anpr = ANPR(debug=True)
                            #(results, lpCnt) = anpr.find_and_ocr(image, psm=7,clearBorder=True)
                            f=open(str(i)+".jpg", "rb") #3.7kiB in same folder
                            fileContent = f.read()
                            byteArr = bytearray(fileContent)
                            client.publish(MQTT_PHOTO,byteArr)
                    except:
                        pass
                    
                                    
                    #print((ObjectDetected[0])["name"])                    
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
                        min_score_thresh=0.20)
                    
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
                                      
                    # get coordinates for the center point
                    x = int(center[0])
                    y = int(center[1]) # for vertical adjustment 
                    
                    # draw the center point
                    cv2.circle(frame,(x,y), 6, (0,0,255), -1)
                    
                    # passing coordinates to adjust the camera
                    mapServoPosition (x,y)
            
            
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
    
    ObjectTrackingProcess = Process(target=ObjectTrackingCamera)
    DistanceProcess = Process(target=distance)
    AutoLightProcess = Process(target=AutoLight)
    ALAM = alamSuspicious()
    GATE = gateController()
    
    process_list = [ObjectTrackingProcess,DistanceProcess,AutoLightProcess]
    for p in process_list:
        print("Process start")
        p.start()
        time.sleep(5)
        
    for p in process_list:
        p.join()

    # Listen for q key
    k = cv2.waitKey(1) & 0xFF
    if k == ord('q'):
        ObjectTrackingProcess.terminate()
        DistanceProcess.terminate()
        AutoLightProcess.terminate()
        
