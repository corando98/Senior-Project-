from time import sleep
import RPi.GPIO as GPIO
import sys

# Transition to PWN
# Truth table functions
# Pins
# Left side
# Pin 11 = IN4L + IN2L
# Pin 13 = ENAL + ENBL
# Right side
# Pin 16 = IN1R + IN3R
# PIN 15 = ENAR + ENBR
# 11| 16| 15  | 13
# L | R | EnR | EnL
# x | x |  0  |  0 -> Off
# 0 | 0 |  1  |  1 -> reverse
# 0 | 1 |  1  |  1 -> left
# 1 | 0 |  1  |  1 -> right
# 1 | 1 |  1  |  1 -> forward
# Note: PWN on enable controls speed
IOActions = [
    ["Right", 23],
    ["Left", 17],
    ["EnableL", 19],  # PWN
    ["EnableR", 13],
]  # PWN

# New pins based on BCM board
# Right: board pin 16 -> BCM GPIO 23
# Left: board pin 11 -> BCM GPIO 17
# EnL: board pin 35 -> BCM GPIO 19
# EnR: board pin 33 -> BCM GPIO 13

GPIO.setmode(GPIO.BCM)
for i in range(4):
    GPIO.setup(IOActions[i][1], GPIO.OUT)

EnL = GPIO.PWM(IOActions[2][1], 50)
EnR = GPIO.PWM(IOActions[3][1], 50)
EnL.start(25)
EnR.start(25)
print("Pins initialized...")


def reset():  # BROKEN
    # Turns enable pin to low
    for i in range(2):
        GPIO.output(IOActions[i][1], GPIO.LOW)


def reverse():
    print("Moving backwards")
    for i in range(2):
        GPIO.output(IOActions[i][1], GPIO.HIGH)


def forward():
    # Turns L and R to low and enable to high
    print("Moving forward")
    for i in range(2):
        GPIO.output(IOActions[i][1], GPIO.LOW)


def turn(direction):
    # Positve moves right, negative moves left
    if direction > 0:
        # Right
        GPIO.output(IOActions[0][1], GPIO.HIGH)
        GPIO.output(IOActions[1][1], GPIO.LOW)
        # Outside wheel moves faster

    else:
        # Left
        GPIO.output(IOActions[0][1], GPIO.LOW)
        GPIO.output(IOActions[1][1], GPIO.HIGH)
        # Outside wheel moves faster


print("\n")
print("The default speed & direction of motor is LOW & Forward.....")
print("r-run s-stop f-forward b-backward l-low m-medium h-high e-exit")
print("\n")
count = 0
while True:
    try:

        x = input()
        if x == "w":
            forward()
        elif x == "s":
            reverse()
        elif x == "a":
            turn(-1)
        elif x == "d":
            turn(1)
        elif x == "l":
            print("low")
            EnR.ChangeDutyCycle(25)
            EnL.ChangeDutyCycle(25)
            x = "z"

        elif x == "m":
            print("medium")
            EnR.ChangeDutyCycle(50)
            EnL.ChangeDutyCycle(50)
            x = "z"

        elif x == "h":
            print("high")
            EnR.ChangeDutyCycle(75)
            EnL.ChangeDutyCycle(75)
            x = "z"
        elif x == "x":
            print("stop")
            EnR.ChangeDutyCycle(0)
            EnL.ChangeDutyCycle(0)
        elif x == "i":  # increase
            count += 5
            EnR.ChangeDutyCycle(count)
            EnL.ChangeDutyCycle(count)
            print("Speed increase to " + str(count))
        elif x == "o":  # increase
            count -= 5
            EnR.ChangeDutyCycle(count)
            EnL.ChangeDutyCycle(count)
            print("Speed decrease to " + str(count))

    except KeyboardInterrupt:
        GPIO.cleanup()
        sys.exit()
