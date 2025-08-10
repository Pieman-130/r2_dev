import time
import RPi.GPIO as GPIO
from board import SCL, SDA
import busio

from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

i2c = busio.I2C(SCL, SDA)
if GPIO.getmode() != 11:
    GPIO.setmode(GPIP.BCM)

#Fwd and Rev pins for LT and RT motors respectively
GPIO.setup(17, GPIO.OUT)
GPIO.setup(27, GPIO.OUT)

pca = PCA9685(i2c)
pca.frequency = 50

start = 0
stop = 3000
ser1 = servo.Servo(pca.channels[14], min_pulse = start, max_pulse = stop)
ser2 = servo.Servo(pca.channels[15], min_pulse = start, max_pulse = stop)

#Set to fwd 17 is left and 27 is right
GPIO.output(17, GPIO.LOW)
GPIO.output(27, GPIO.HIGH)

x = input("Press ENTER to Start.")
ser2.angle = 180
ser1.angle = 180
x = input("Press ENTER to Pause")
ser2.angle = 0
ser1.angle = 0
x = input("Press ENTER to reverse")

print("Reverse")
GPIO.output(17, GPIO.HIGH)
GPIO.output(27, GPIO.LOW)
ser1.angle = 90
ser2.angle = 90

x = input("Press ENTER to STOP")
ser1.angle = 0
ser2.angle = 0

GPIO.cleanup()


