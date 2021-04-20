import paho.mqtt.client as mqtt
import time
amazon = "18.212.25.214"
pi = "192.168.8.196"
MQTT_BROKER =  pi
MQTT_SEND_ULTRASONIC = "home/distance"

class ultraSonicDistanceMQTT:
        # The callback for when the client receives a CONNACK response from the server.
        def __init__(self):
            global distance
            self.distance = ""
        
        def on_connect(client, userdata, flags, rc):
            #print("Connected with result code "+str(rc))

            # Subscribing in on_connect() means that if we lose the connection and
            # reconnect then subscriptions will be renewed.
            client.subscribe(MQTT_SEND_ULTRASONIC)


        # The callback for when a PUBLISH message is received from the server.
        def on_message(client, userdata, msg):
            global distance
            # Decoding the message
            distance = msg.payload
            

        client = mqtt.Client("Distance")

        client.connect(MQTT_BROKER,1883)
        client.subscribe(MQTT_SEND_ULTRASONIC)
        client.on_connect = on_connect
        client.on_message = on_message

        # Starting thread which will receive the frames
        client.loop_start()
        
        def get_distance():
            global distance
            if distance:
                return distance
            else:
                return 0
        # Stop the Thread
        # client.loop_stop()
        

