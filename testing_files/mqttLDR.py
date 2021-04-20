import paho.mqtt.client as mqtt
import time
import copy

amazon = "18.212.25.214"
pi = "192.168.8.196"
MQTT_BROKER =  pi
MQTT_SEND_LIGHT_INTENSITY = "home/light"

class LDR_MQTT:
        global client

        #The callback for when the client receives a CONNACK response from the server.
        def __init__(self):
            global resistance
            self.resistance = ""
        
        def on_connect(client, userdata, flags, rc):
            #print("Connected with result code "+str(rc))

            # Subscribing in on_connect() means that if we lose the connection and
            # reconnect then subscriptions will be renewed.
            client.subscribe(MQTT_SEND_LIGHT_INTENSITY)


        # The callback for when a PUBLISH message is received from the server.
        def on_message(client, userdata, msg):
            global resistance
            # Decoding the message
            resistance = msg.payload
            
        
        client = mqtt.Client("LDR")
        client.connect(MQTT_BROKER,1883)
        client.subscribe(MQTT_SEND_LIGHT_INTENSITY)
        client.on_connect = on_connect
        client.on_message = on_message

        

        # Starting thread which will receive the frames
        client.loop_start()
        client.on_message = on_message

        def get_resistance():
            global resistance
            global client

            if resistance:
                client.publish("home/LED",copy.copy(resistance))
                return resistance
            return 0
        


