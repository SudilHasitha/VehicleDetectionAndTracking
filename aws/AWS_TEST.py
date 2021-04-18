import paho.mqtt.client as mqtt
import ssl
import json
import time

def on_connect(client,userdata,flags,rc):
    print("connection results: "+str(rc))
    
def on_message(client,userdata,msg):
    print(msg.topic+" "+str(msg.payload))

# Topic on which frame will be published
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

################# AWS connection credentials ######################
awshost = "a21vdoiadhttuh-ats.iot.us-east-1.amazonaws.com"
awsport=8883
clientId = "VehicleDetector"
thingName = "VehicleDetector"
caPath = "root-CA.crt"
certPath = "VehicleDetector.cert.pem"
keyPath = "VehicleDetector.private.key"

################### connect to aws ###############################
client.tls_set(caPath,certfile=certPath,keyfile=keyPath,cert_reqs=ssl.CERT_REQUIRED,tls_version=ssl.PROTOCOL_TLSv1_2, ciphers=None)
client.connect(awshost,awsport,keepalive=60)
client.loop_start()
while True:
    payload = json.dumps({"message":"1234 you"})
    client.publish("MQTT_SEND_AWS",payload,qos=0)
    time.sleep(1)