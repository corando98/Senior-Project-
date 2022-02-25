import time
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

print("Pins initialized...")


def reset():  # BROKEN
    # Turns enable pin to low
    for i in range(2):
        GPIO.output(IOActions[i][1], GPIO.LOW)


def reverse():
    print("Moving backwards")
    for i in range(2):
        GPIO.output(IOActions[i][1], GPIO.LOW)


def forward():
    # Turns L and R to low and enable to high
    print("Moving forward")
    for i in range(2):
        GPIO.output(IOActions[i][1], GPIO.HIGH)


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
    # Sleep based turns, 8 = 180, 16 = 90, 32 = 45, more or less
    time.sleep(abs(direction / SpeedMotor / 4))
    for i in range(2):
        GPIO.output(IOActions[i][1], GPIO.LOW)


# Ultrasonic sensor setup
TRIG = 2
ECHO = 3

GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
GPIO.output(TRIG, False)
print("Waiting for sensor to settle")
time.sleep(2)

SpeedMotor = 20  # 0-100 range
EnL.start(SpeedMotor)
EnR.start(SpeedMotor)


def EchoDist():
    GPIO.output(TRIG, True)  # Sends pulse to trigger
    time.sleep(0.00001)
    GPIO.output(TRIG, False)
    pulse_start = 0
    pulse_end = 0
    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()

    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    distance = round(distance, 2)
    print("Distance:", distance, "cm")
    return distance


lastTurn = "right"
lastDistCheck = []
DistanceThreshold = 40
while True:
    try:
        if EchoDist() > DistanceThreshold:
            forward()
            print("Last turn: ", lastTurn)
        # Snake navigation mode
        else:
            # identify corner stuck
            if len(lastDistCheck) > 3:
                if (
                    lastDistCheck[0] < DistanceThreshold
                    and lastDistCheck[1] < DistanceThreshold
                    and lastDistCheck[2] < DistanceThreshold
                ):
                    print("Corner stuck", lastDistCheck)
                    turn(180)

            if lastTurn == "left":
                while EchoDist() < DistanceThreshold:
                    turn(90)
                lastTurn = "right"
                lastDistCheck.append(EchoDist())
            elif lastTurn == "right":
                while EchoDist() < DistanceThreshold:
                    turn(-90)
                lastTurn = "left"
                lastDistCheck.append(EchoDist())

    except KeyboardInterrupt:
        GPIO.cleanup()
        sys.exit()
