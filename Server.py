#!/usr/bin/env python
from flask import Flask, render_template, Response
from picamera import PiCamera
from picamera.array import PiRGBArray
import cv2
import numpy as np

app = Flask(__name__)

@app.route('/')
def index():
    return render_template('/index.html')

def gen():
    # Initialize frame rate calculation
    frame_rate_calc = 1
    freq = cv2.getTickFrequency()
    font = cv2.FONT_HERSHEY_SIMPLEX
    
    camera = PiCamera()
    camera.resolution = (640,480)
    camera.framerate = 10
    rawCapture = PiRGBArray(camera, size=(640,480))
    rawCapture.truncate(0)
        
    for frame1 in camera.capture_continuous(rawCapture, format="bgr",use_video_port=True):
        t1 = cv2.getTickCount()
        
        # Acquire frame and expand frame dimensions to have shape: [1, None, None, 3]
        # i.e. a single-column array, where each item in the column has the pixel RGB value

        frame = np.copy(frame1.array)
        frame.setflags(write=1)
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame_expanded = np.expand_dims(frame_rgb, axis=0)
        cv2.imshow("Object Detection",frame)        

@app.route('/video_feed')
def video_feed():
    return Response(gen(),mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='192.168.8.196',port=5000, debug=True)