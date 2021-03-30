import paho.mqtt.client as mqtt
import time

MQTT_BROKER = "192.168.8.196"
MQTT_SEND_LIGHT_INTENSITY = "home/light"

class LDR_MQTT:
        # The callback for when the client receives a CONNACK response from the server.
        def __init__(self):
            global resistance
            self.resistance = ""
        
        def on_connect(client, userdata, flags, rc):
            print("Connected with result code "+str(rc))

            # Subscribing in on_connect() means that if we lose the connection and
            # reconnect then subscriptions will be renewed.
            client.subscribe(MQTT_SEND_LIGHT_INTENSITY)


        # The callback for when a PUBLISH message is received from the server.
        def on_message(client, userdata, msg):
            global resistance
            # Decoding the message
            resistance = msg.payload
        
        client = mqtt.Client()
        client.on_connect = on_connect
        client.on_message = on_message

        client.connect(MQTT_BROKER)

        # Starting thread which will receive the frames
        client.loop_start()
        time.sleep(0.1)
        def get_resistance():
            global resistance
            return resistance
        # Stop the Thread
        # client.loop_stop()
        

