import paho.mqtt.client as mqtt

mqtt_username    = "pi"
mqtt_password    = "raspberry"
amazon = "18.212.25.214"
pi = "192.168.8.196"
MQTT_BROKER =  pi
MQTT_GATE = "home/gate"

## gate controlling code
from gateControls import gateControls
        
class gateController:
            
            # The callback for when the client receives a CONNACK response from the server.
            def on_subscribe(client, userdata, mid, granted_qos):   #create function for callback
                print("subscribed with qos",granted_qos, "\n")
                
            def on_disconnect(client, userdata,rc=0):
                print("DisConnected result code "+str(rc))

            def on_connect(client, userdata, flags, rc):
                print("Connected flags"+str(flags)+"result code "+str(rc))


            def on_message(client, userdata, message):
                msg=str(message.payload.decode("utf-8"))
                gateObj = gateControls()

                if int(msg) > 50:
                    gateObj.moveSteps(1,3,512)
                if int(msg) <50:
                    gateObj.moveSteps(0,3,512)
                print("message received  "  +msg)
                
            def on_publish(client, userdata, mid):
                print("message published "  +str(mid))

            send_topic ="house/client_b/" +MQTT_GATE
            client= mqtt.Client("ClientA",False)       #create client object

            client.on_subscribe = on_subscribe   #assign function to callback
            client.on_disconnect = on_disconnect #assign function to callback
            client.on_connect = on_connect #assign function to callback
            client.on_message=on_message
            client.connect(MQTT_BROKER,1883)           #establish connection
            client.loop_start()
            client.subscribe(MQTT_GATE)
                
                

