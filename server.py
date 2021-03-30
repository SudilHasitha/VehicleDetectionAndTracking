
from flask import Flask, render_template, Response
import base64
import cv2 as cv
import numpy as np
import paho.mqtt.client as mqtt
import time
import io
from PIL import Image

#get mqtt classes
from mqttvideostream import videoStreamMQTT
from mqttultrasonic import ultraSonicDistanceMQTT
from mqttLDR import LDR_MQTT

#Initialize the Flask app
app = Flask(__name__)

def genframe(mqttvideostream):
    while True:
        frame = mqttvideostream.get_frame()
        yield (b'--frame\r\n'
            b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')

def getDistance(mqttultrasonic):
    while True:
        distance = mqttultrasonic.get_distance()
        yield distance

def getResistance(mqttLDR):
    while True:
        resistance = mqttLDR.get_resistance()
        yield resistance


@app.route('/')
def index():
    return render_template('index.html',LDR_feed= next(getResistance(LDR_MQTT)),distance_feed=next(getDistance(ultraSonicDistanceMQTT)))

@app.route('/video_feed')
def video_feed():
    return Response(genframe(videoStreamMQTT), mimetype='multipart/x-mixed-replace; boundary=frame')

# @app.route('/distance_feed')
# def distance_feed():
#     return Response(getDistance(ultraSonicDistanceMQTT),mimetype='text')

# @app.route('/LDR_feed')
# def LDR_feed():
#     return Response(getResistance(LDR_MQTT),mimetype='text')


if __name__ == "__main__":
        app.run(debug=True)
        