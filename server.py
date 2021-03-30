
from flask import Flask, render_template, Response
import base64
import cv2 as cv
import numpy as np
import paho.mqtt.client as mqtt
import time
import io
from PIL import Image
from mqttvideostream import videoStreamMQTT

#Initialize the Flask app
app = Flask(__name__)

def genframe(mqttvideostream):
        while True:
            frame = mqttvideostream.get_frame()
            yield (b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    # # Stop the Thread
    # client.loop_stop()
    return Response(genframe(videoStreamMQTT), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == "__main__":
        app.run(debug=True)
        