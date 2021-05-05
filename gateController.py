import paho.mqtt.client as mqtt
import time
import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)
control_pins = [7,25,5,6]

amazon = "18.212.25.214"
pi = "192.168.8.196"
MQTT_BROKER =  pi
MQTT_SEND_ULTRASONIC = "home/gate"

class gateController:
        # The callback for when the client receives a CONNACK response from the server.
        def __init__(self):
            global distance
            self.distance = ""
        
        def on_connect(self,client, userdata, flags, rc):
            print("Connected with result code "+str(self.rc))

            # Subscribing in on_connect() means that if we lose the connection and
            # reconnect then subscriptions will be renewed.
            client.subscribe(MQTT_SEND_ULTRASONIC)


        # The callback for when a PUBLISH message is received from the server.
        def on_message(client, userdata, msg):
            global gate
            # Decoding the message
            gate = msg.payload
            print(gate)
            

        client = mqtt.Client("gate")

        client.connect(MQTT_BROKER,1883)
        client.subscribe(MQTT_SEND_ULTRASONIC)
        client.on_connect = on_connect
        client.on_message = on_message

        # Starting thread which will receive the frames
        client.loop_start()
        
        def gateState(self):
            global gate
            if gate:
                    for pin in control_pins:
                        GPIO.setup(pin, GPIO.OUT)
                        GPIO.output(pin, 0)
                    halfstep_seq = [
                      [1,0,0,0],
                      [1,1,0,0],
                      [0,1,0,0],
                      [0,1,1,0],
                      [0,0,1,0],
                      [0,0,1,1],
                      [0,0,0,1],
                      [1,0,0,1]
                    ]
                    for i in range(512):
                        for halfstep in range(8):
                                for pin in range(4):
                                    GPIO.output(control_pins[pin], halfstep_seq[halfstep][pin])
                                time.sleep(0.001)
            else:
                return 0
        


