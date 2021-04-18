import paho.mqtt.client as mqtt    #import client library
def on_connect(client, userdata, flags, rc):
   if rc==0:
      print("connected ok")
client = mqtt.Client("python1")             #create new instance 
client.on_connect=on_connect  #bind call back function
client.connect("54.175.9.28")               #connect to broker
client.loop_start()  #Start loop 
time.sleep(4) # Wait for connection setup to complete

client.loop_stop()    #Stop loop