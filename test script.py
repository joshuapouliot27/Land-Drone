import RPi.GPIO as GPIO
from electronics.gateways import LinuxDevice
from electronics.gateways import LinuxDevice
import time
GPIO.setmode(GPIO.BOARD)

TRIG = 18
ECHO = 22

gw = LinuxDevice(1)

# BUTTON TEST

# MAGNOMETER TEST

# SONAR TEST
# print("Distance Measurement In Progress")
#
# GPIO.setup(TRIG,GPIO.OUT)
# GPIO.setup(ECHO,GPIO.IN)
#
# GPIO.output(TRIG, False)
# print("Waiting For Sensor To Settle")
# time.sleep(0.05)
#
# GPIO.output(TRIG, True)
# time.sleep(0.00001)
# GPIO.output(TRIG, False)
#
# while GPIO.input(ECHO)==0:
#     pulse_start = time.time()
# while GPIO.input(ECHO)==1:
#     pulse_end = time.time()
#
# pulse_duration = pulse_end - pulse_start
#
# distance = (pulse_duration / 2) * 1125.33
#
# distance = round(distance, 2)
#
# print("Distance:"+str(distance)+"ft")

GPIO.cleanup()


