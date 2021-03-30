import paho.mqtt.client as mqtt
import time

MQTT_BROKER = "192.168.8.196"
MQTT_SEND_ULTRASONIC = "home/distance"

class ultraSonicDistanceMQTT:
        # The callback for when the client receives a CONNACK response from the server.
        def __init__(self):
            global distance
            self.distance = ""
        
        def on_connect(client, userdata, flags, rc):
            print("Connected with result code "+str(rc))

            # Subscribing in on_connect() means that if we lose the connection and
            # reconnect then subscriptions will be renewed.
            client.subscribe(MQTT_SEND_ULTRASONIC)


        # The callback for when a PUBLISH message is received from the server.
        def on_message(client, userdata, msg):
            global distance
            # Decoding the message
            distance = msg.payload
        
        client = mqtt.Client()
        client.on_connect = on_connect
        client.on_message = on_message

        client.connect(MQTT_BROKER)

        # Starting thread which will receive the frames
        client.loop_start()
        time.sleep(0.1)
        def get_distance():
            global distance
            return distance
        # Stop the Thread
        # client.loop_stop()
        

