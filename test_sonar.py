import RPi.GPIO as GPIO
import time

sonar_trig_pin = 18
sonar_echo_pin = 22
time_start = 0
time_end = 0

GPIO.setmode(GPIO.BOARD)

GPIO.setup(sonar_echo_pin, GPIO.IN)
GPIO.setup(sonar_trig_pin, GPIO.OUT)
GPIO.output(sonar_trig_pin, False)

GPIO.output(sonar_trig_pin, False)
time.sleep(.05)

GPIO.output(sonar_trig_pin, True)
time.sleep(.000001)
GPIO.output(sonar_trig_pin, False)

while GPIO.input(sonar_echo_pin) == 0:
    time_start = time.time()
while GPIO.input(sonar_echo_pin) == 1:
    time_end = time.time()

total_time = time_end - time_start
distance = (total_time / 2) * 1125.33  # Calculated in ft/s
print("Sonar distance: {0:.2} ft".format(distance))