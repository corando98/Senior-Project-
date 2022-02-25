from random import randrange, uniform
from time import sleep
import time
import paho.mqtt.client as mqtt
from threading import Thread
import RPi.GPIO as GPIO


# variables to split payload
data = "empty"
distance = 0.0
motor_direction = "none"
action = False
obj_xPos = 0
obj_yPos = 0

avgY = 0.0
yCord = [1, 2, 3]
avgX = 0.0
xCord = [1, 2, 3]

startTime = time.time()
setSleep = False
detectionModel = "trash"
enableModel = True
# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe("test")
    client.subscribe("model")


# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    global data
    global detectionModel
    global motor_direction
    global obj_xPos
    global obj_yPos
    global avgX
    global avgY
    global yCord
    global xCord
    global action
    global distance
    global startTime
    global setSleep
    # print(str(msg.payload))
    mqtt_in = str(msg.payload)
    if mqtt_in.find("test") != -1:
        print("Stop")
    if mqtt_in.find("litter") != -1:
        pos1 = mqtt_in.find("[")
        pos2 = mqtt_in.find(",")
        # ObjCenterX
        obj_xPos = float(mqtt_in[(pos1 + 1) : pos2])
        pos1 = mqtt_in.find("]")
        # ObjCenterY
        obj_yPos = float(mqtt_in[(pos2 + 2) : pos1])

        # Find average of X and Y values of obj position
        # objX->1 1->2 2->3
        # Smoothing
        yCord[2] = yCord[1]
        yCord[1] = yCord[0]
        yCord[0] = obj_yPos
        avgY = (yCord[2] + yCord[1] + yCord[0]) / 3

        xCord[2] = xCord[1]
        xCord[1] = xCord[0]
        xCord[0] = obj_xPos
        avgX = (xCord[2] + xCord[1] + xCord[0]) / 3

        print("Average x" + str(avgX))
        print("Average Y" + str(avgY))

    # 320x320
    # Half = 160

    if avgX == 0:
        motor_direction = "none"
    elif avgX > 192:
        motor_direction = "right"
    elif avgX < 128:
        motor_direction = "left"
    elif avgX < 192 and avgX > 128:
        motor_direction = "straight"


client = mqtt.Client()
client.on_connect = on_connect
client.connect("localhost", 1883, 60)


def runThread():
    client.on_message = on_message
    client.loop_forever()


if __name__ == "__main__":
    t2 = Thread(target=runThread)
    t2.setDaemon(True)
    t2.start()

    old = data
    sleep(1)
    while True:
        if setSleep == False:
            if detectionModel == "trash":
                # Gives 10 seconds before moving
                if time.time() > startTime + 10:
                    motor_direction = "none"
                if motor_direction == "left":
                    print("moving left")
                elif motor_direction == "right":
                    print("moving right")
