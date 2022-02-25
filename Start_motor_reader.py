import paho.mqtt.client as mqtt
import time
from time import sleep
from threading import Thread

import motor_controller as motor

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

faceSize = 0
startFaceDistance = 0
startFace = False

override = False
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
    global override
    global startDistanceFace
    global startFace
    global faceSize
    mqtt_in = str(msg.payload)
    if mqtt_in.find("motor") != -1:
        detectionModel = "manual"
        if mqtt_in.find("forward") != -1:
            motor_direction = "straight"
            override = True
        if mqtt_in.find("reverse") != -1:
            motor_direction = "reverse"
            override = True
        if mqtt_in.find("reset") != -1:
            motor_direction = "none"
            override = True
        if mqtt_in.find("right") != -1:
            motor_direction = "right"
            override = True
        if mqtt_in.find("left") != -1:
            override = True
            motor_direction = "left"
        print("Current motor direction:" + motor_direction)
        if mqtt_in.find("detect") != -1:
            override = False
            print("Manual override off")

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

        # print("Average x" + str(avgX))
        # print("Average Y" + str(avgY))

    # 320x320
    # Half = 160
    print("AvgX:", avgX)
    if override == False:
        if avgX == 0:
            motor_direction = "none"
        elif avgX > 224:
            motor_direction = "right"
        elif avgX < 160:
            motor_direction = "left"
        elif 160 < avgX and avgX < 224:
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

    delay = 0
    while True:
        try:
            if setSleep == False:
                if (
                    detectionModel == "trash"
                ):  ####################### Trash detection actions
                    # Gives x seconds before moving (4s)
                    # if time.time() < startTime + 4:
                    #     motor_direction = "none"
                    # # No motor_direction, scan in a circle
                    # if motor_direction == "none":
                    #     turn(-1)  # Turns left for x seconds
                    ################################################# Initial boot up actions above.
                    if motor_direction == "left":
                        motor.turn(-2)
                    elif motor_direction == "right":
                        motor.turn(1.5)
                    elif motor_direction == "straight":
                        motor.forward()
                    elif motor_direction == "reverse":
                        motor.reverse()
                    # sleep(0.5)
                    ############################ Time between motor actions

                elif detectionModel == "manual":
                    if motor_direction == "none":
                        motor.reset()
                    if motor_direction == "left":
                        motor.turn(-2)
                    elif motor_direction == "right":
                        motor.turn(1.5)
                    elif motor_direction == "straight":
                        motor.forward()
                    elif motor_direction == "reverse":
                        motor.reverse()
                    sleep(0.2)
                    ############################ Time between motor actions
            ######################## DEBUGGING #####################
            ########## ATTEMPT AT DELAY  #############
            if (delay + 0.5) < time.time():
                delay = time.time()
                print(
                    "\nDetection model: "
                    + detectionModel
                    + "\nMotor direction: "
                    + motor_direction
                    + "\n Time: "
                    + str(time.time())
                )

        except KeyboardInterrupt:
            print("\nStopping")
            reset()
            GPIO.cleanup()
            sys.exit()


# Cleanup before Exit
reset()
GPIO.cleanup()
