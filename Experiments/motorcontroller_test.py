import RPi.GPIO as GPIO
# replace for keyboard for testing on device
import keyboard
import time

# Position, motor pin (OUT), sensor 1 (IN), sensor 2 (IN)

IOActions = [["FWleft", 13, 0, 15], #White N1
             ["RWleft", 15, 18, 22], #Yellow N2
             ["FWright", 16, 33, 35], #Green N3
             ["RWright", 18, 38, 40]] #Blue N4


GPIO.setmode(GPIO.BOARD)
# Setup Pins:
for i in range(4):
    GPIO.setup(IOActions[i][1], GPIO.OUT)

    # Placeholder for encoder pins
    # GPIO.setup(IOActions[i][2], GPIO.IN)
    # GPIO.setup(IOActions[i][3], GPIO.IN)
    
print('Pins initialized...')

# Motion Functions
def reset():
    for i in range(4):
        GPIO.output(IOActions[i][1], GPIO.LOW)
    print('Setting all motors to LOW')

def reverse():
    #Moves left and right side of the motors forward
    
    GPIO.output(IOActions[0][1], GPIO.LOW)
    GPIO.output(IOActions[1][1], GPIO.HIGH)
    GPIO.output(IOActions[2][1], GPIO.HIGH)
    GPIO.output(IOActions[3][1], GPIO.LOW)
    
def forward():
    #Moves left and right side of the motors forward
    GPIO.output(IOActions[0][1], GPIO.HIGH)
    GPIO.output(IOActions[1][1], GPIO.LOW)
    GPIO.output(IOActions[2][1], GPIO.LOW)
    GPIO.output(IOActions[3][1], GPIO.HIGH)
def turn(direction):
    
    # direction negative is left, positive is right
    if(direction > 0):
        print('Right')
        GPIO.output(IOActions[0][1], GPIO.HIGH)
        GPIO.output(IOActions[1][1], GPIO.LOW)
        GPIO.output(IOActions[2][1], GPIO.HIGH)
        GPIO.output(IOActions[3][1], GPIO.LOW)
        
    elif (direction < 0):
        print('Left')
        GPIO.output(IOActions[0][1], GPIO.LOW)
        GPIO.output(IOActions[1][1], GPIO.HIGH)
        GPIO.output(IOActions[2][1], GPIO.LOW)
        GPIO.output(IOActions[3][1], GPIO.HIGH)

def MotorCheck(N):
    GPIO.output(IOActions[N][1], GPIO.HIGH)


reset()
loop = True
while(loop):
    #Delays loop to wait for next key down event
    keyboard.read_key('')
    
    if keyboard.is_pressed('w'):
       forward()

    if keyboard.is_pressed('s'):
       reverse()

    if keyboard.is_pressed('r'):
        reset()

    if keyboard.is_pressed('d'):
        turn(1)

    if keyboard.is_pressed('a'):
        turn(-1)
 
    if keyboard.is_pressed('m'):
        test()
    if keyboard.is_pressed('x'):
        loop = False
        
    
    if keyboard.is_pressed('u'):
        MotorCheck(0) #ForwardLeft
    if keyboard.is_pressed('i'):
        MotorCheck(1) #ReverseLeft
    if keyboard.is_pressed('j'):
        MotorCheck(2) #ReverseRight
    if keyboard.is_pressed('k'):
        MotorCheck(3) #ForwardRight


# Cleanup before Exit
GPIO.cleanup()
