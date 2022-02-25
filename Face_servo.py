import RPi.GPIO as GPIO
from time import sleep

GPIO.setmode(GPIO.BOARD)

GPIO.setup(3, GPIO.OUT)
pwm = GPIO.PWM(3, 50)  # 50Hz on pin 3
pwm.start(0)  # Start on duty cycle 0


def SetAngle(angle):
    duty = angle / 18 + 2
    GPIO.output(3, True)
    pwm.ChangeDutyCycle(duty)
    sleep(1)
    GPIO.output(3, False)
    pwm.ChangeDutyCycle(0)
