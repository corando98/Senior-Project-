import tkinter as tk
import RPi.GPIO as GPIO
import time
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
TRIGPIN = 11
ECHO = 15 

print ("Distance Measurement In Progress")
GPIO.setup(TRIGPIN,GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN, pull_up_down= GPIO.PUD_DOWN)

root = tk.Tk()
root.title("Calculating Distance")

GPIO.output(TRIGPIN, False)
print ("Delay for sensor stability")
time.sleep(2)
GPIO.output(TRIGPIN, True)
time.sleep(0.00001)
GPIO.output(TRIGPIN, False)
while GPIO.input(ECHO)==0:
    pulse_start= 0
    pulse_start= time.time()
while GPIO.input(ECHO)==1:
    pulse_end= 0
    pulse_end= time.time()
duration = pulse_end-pulse_start

distance = duration * 34029
distance = distance / 2
distance = round(distance, 2) 
print ("Distance:",distance,"cm")

    
label = tk.Label(root,width=40, fg="yellow" , bg = "purple")
label.config(font=("Courier", 36))
label.config(text=str(distance))
label.pack()
button = tk.Button(root, text='Stop', width=50, command=root.destroy)
button.pack()
root.mainloop()
GPIO.cleanup()