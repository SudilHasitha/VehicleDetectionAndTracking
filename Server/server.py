
from flask import Flask, render_template, Response, request,jsonify
import sqlite3 as sql
#get mqtt classes
from mqttvideostream import videoStreamMQTT
#from mqttvideostream2 import videoStreamMQTT
from mqttultrasonic import ultraSonicDistanceMQTT
from mqttLDR import LDR_MQTT
from mqttphoto import photoMQTT
# from mqttgate import gateMQTT
import paho.mqtt.client as mqtt

amazon = "18.212.25.214"
pi = "192.168.8.196"
MQTT_BROKER =  pi
MQTT_GATE = "home/gate"


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
    con = sql.connect("database.db")
    con.row_factory = sql.Row
   
    cur = con.cursor()
    cur.execute("select * from vehicles")
   
    rows = cur.fetchall();
    print(rows)
    return render_template('index.html',rows=rows)

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


@app.route('/addrec',methods = ['POST'])
def addrec():
        try:
            nm = request.form['number']
            print(nm)
            con = sql.connect("database.db",timeout=15)
            print(con)
            cur = con.cursor()
            cur.execute('''INSERT INTO vehicles VALUES (?)''',(nm,))
            con.commit()
            print("Record successfully added")
            return jsonify({"Success":"Record Added"})
            con.close()
        except :
            con.rollback()
            print("error in insert operation")
            return jsonify({"Error":"Record Not Added"})

@app.route('/index_get_data')
def getdata():
    con = sql.connect("database.db")
    con.row_factory = sql.Row
   
    cur = con.cursor()
    cur.execute("select * from vehicles")
   
    rows = cur.fetchall();
    print(rows)
    return Response(rows=rows)

@app.route("/lightIntensity", methods=['GET'])
def getData():

    intensity = request.args.get('intensity')
    # gateMQTTObj = gateMQTT()
    # # gateMQTTObj.gateState(intensity)
    # gateMQTTObj.intensity = intensity
    client = mqtt.Client("Gate")
    client.connect(MQTT_BROKER,1883)
    client.loop_start()
    client.publish(MQTT_GATE,intensity)
    return jsonify({"Success":"Record Added"})

if __name__ == "__main__":
    app.run(debug=True)
        