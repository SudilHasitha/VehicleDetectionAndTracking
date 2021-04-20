
from flask import Flask, render_template, Response

#get mqtt classes
from mqttvideostream import videoStreamMQTT
#from mqttvideostream2 import videoStreamMQTT
from mqttultrasonic import ultraSonicDistanceMQTT
from mqttLDR import LDR_MQTT
from mqttphoto import photoMQTT

#Initialize the Flask app
app = Flask(__name__)
global distance,intensity
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

def getPhoto(mqttphoto):
    while True:
        frame = mqttphoto.get_photo()
        yield (b'--frame\r\n'
            b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')

@app.route('/')
def index():
    global distance,intensity
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    return Response(genframe(videoStreamMQTT), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/distance_feed')
def distance_feed():
    global distance
    distance = next(getDistance(ultraSonicDistanceMQTT))
    return Response(distance)

@app.route('/LDR_feed')
def LDR_feed():
    global intensity
    intensity = next(getResistance(LDR_MQTT))
    return Response(intensity)

@app.route('/get_photo')
def get_photo():
    return Response(getPhoto(photoMQTT), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == "__main__":
    app.run(debug=False)
        