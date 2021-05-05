#!/usr/bin/env python
import paho.mqtt.client as mqtt
import RPi.GPIO as GPIO
import time, threading
amazon = "18.212.25.214"
pi = "192.168.8.196"
MQTT_BROKER =  pi
MQTT_ALAM = "home/ALAM"

class alamSuspicious:
        
    def on_connect(self,client, userdata, flags, rc):
        print("Connected with result code "+str(self.rc))
        # Subscribing in on_connect() means that if we lose the connection and
        # reconnect then subscriptions will be renewed.
        print(client.subscribe(MQTT_ALAM))
        BUZZER = 12
        GPIO.setmode(GPIO.BCM)
        if rc:
            GPIO.setup(BUZZER, GPIO.OUT)
            GPIO.output(BUZZER,False)
        else:
            GPIO.setup(BUZZER, GPIO.OUT)
            GPIO.output(BUZZER, True)
            time.sleep(0.5)
            GPIO.output(BUZZER,False)
            
            
        client.subscribe(MQTT_ALAM)


    # The callback for when a PUBLISH message is received from the server.
    def on_message(client, userdata, msg):
        print(msg)
        global res
        res = msg.payload
        
            
    client = mqtt.Client("ALAM")
    client.connect(MQTT_BROKER,1883)
    client.on_connect = on_connect
    client.subscribe(MQTT_ALAM)
    client.on_message = on_message
    # Starting thread which will receive the frames
    client.loop_start()
    client.subscribe(MQTT_ALAM)
    client.on_message = on_message
