import sys
from random import randrange, uniform
from time import sleep


try:

    import RPi.GPIO as GPIO

except:
    import Mock.GPIO as GPIO


# replace for keyboard for testing on device #import keyboard

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


# New pins based on BCM board
# Right: board pin 16 -> BCM GPIO 23
# Left: board pin 11 -> BCM GPIO 17
# EnL: board pin 35 -> BCM GPIO 19
# EnR: board pin 33 -> BCM GPIO 13

# PinActions
IOActions = [
    ["Right", 23],
    ["Left", 17],
    ["EnableL", 19],  # PWN
    ["EnableR", 13],  # PWN
]

GPIO.setmode(GPIO.BCM)
for i in range(4):
    GPIO.setup(IOActions[i][1], GPIO.OUT)


EnL = GPIO.PWM(IOActions[2][1], 50)
EnR = GPIO.PWM(IOActions[3][1], 50)
EnL.start(25)
EnR.start(25)
print("Pins initialized...\nStarting motor duty cycle at 25%")


def reset():
    # Turns enable pin to low
    for i in range(2):
        GPIO.output(IOActions[i][1], GPIO.LOW)
        EnR.ChangeDutyCycle(0)
        EnL.ChangeDutyCycle(0)


def reverse():
    print("Moving backwards")
    for i in range(2):
        GPIO.output(IOActions[i][1], GPIO.HIGH)

    sleep(0.2)


def forward():
    # Turns L and R to low and enable to high
    print("Moving forward")
    for i in range(2):
        GPIO.output(IOActions[i][1], GPIO.LOW)
    EnR.ChangeDutyCycle(25)
    EnL.ChangeDutyCycle(25)
    sleep(0.2)


def turn(direction):
    # Posive moves right, negative moves left
    if direction > 0:
        # print("Turning left by ", direction)
        GPIO.output(IOActions[0][1], GPIO.HIGH)
        GPIO.output(IOActions[1][1], GPIO.LOW)

    else:
        # print("Turning right by ", direction)
        GPIO.output(IOActions[0][1], GPIO.LOW)
        GPIO.output(IOActions[1][1], GPIO.HIGH)

    EnR.ChangeDutyCycle(25)
    EnL.ChangeDutyCycle(25)
    sleep(0.0001)
    reset()  # Spins for x amount
