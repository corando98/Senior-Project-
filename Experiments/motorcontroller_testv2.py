import RPi.GPIO as GPIO
# replace for keyboard for testing on device
import keyboard
import time

# L | R | E
# 0 | 0 | 1 = Forward
# 0 | 1 | 1 = Right turn
# 1 | 0 | 1 = Left turn
# 1 | 1 | 1 = Reverse
# x | x | 0 = stop

# PinActions
IOActions = [['Left', 3],
             ['Right', 5],
             ['Enable', 7]]

GPIO.setmode(GPIO.BOARD)
for i in range(3):
    GPIO.setup(IOActions[i][1], GPIO.OUT)

print('Pins initialized...')

def reset():
    print('Turning off controllers')
    #Turns enable pin to low
    for i in range(3):
        GPIO.output(IOActions[i][1], GPIO.LOW)

def reverse():
    print('Moving backwards')
    for i in range(2):
        GPIO.output(IOActions[i][1], GPIO.HIGH)
    GPIO.output(IOActions[2][1], GPIO.HIGH)

def forward():
    # Turns L and R to low and enable to high
    print('Moving forward')
    for i in range(2):
        GPIO.output(IOActions[i][1], GPIO.LOW)
    GPIO.output(IOActions[2][1], GPIO.HIGH)

def turn(direction):
    #Posive moves right, negative moves left
    if direction > 0:
        print('Turning right by ', direction)
        GPIO.output(IOActions[0][1], GPIO.LOW)
        GPIO.output(IOActions[1][1], GPIO.HIGH)
        GPIO.output(IOActions[2][1], GPIO.HIGH)
    else:
        print('Turning left by ', direction)
        GPIO.output(IOActions[0][1], GPIO.HIGH)
        GPIO.output(IOActions[1][1], GPIO.LOW)
        GPIO.output(IOActions[2][1], GPIO.HIGH)



loop = True
reset()
while(loop):
    keyboard.read_key()
    
    if keyboard.is_pressed('w'):
        forward()
    if keyboard.is_pressed('a'):
        turn(-1)
    if keyboard.is_pressed('d'):
        turn(1)
    if keyboard.is_pressed('s'):
        reverse()
    if keyboard.is_pressed('r'):
        reset()
    if keyboard.is_pressed('z'):
        print('Stopping...')
        loop = False
