import paho.mqtt.client as mqtt
import time
amazon = "18.212.25.214"
pi = "192.168.8.196"
MQTT_BROKER =  pi
MQTT_GATE = "home/gate"
client = mqtt.Client("Gate")
client.connect(MQTT_BROKER,1883)
class gateMQTT:
        def on_connect(self,client, userdata, flags, rc):
            pass
            #print("Connected with result code "+str(self.rc))

        def gateState(self,intensity):
            # if intensity > 50:
            #     state = "close"
            # else:
            #     state = "open"
            client.publish(MQTT_GATE,intensity)

        client = mqtt.Client("Gate")
        client.connect(MQTT_BROKER,1883)

        client.loop_start()
        client.on_connect = on_connect
        client.gateState = gateState
        
        