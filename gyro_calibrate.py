#!/usr/bin/python
#   This program is used to calibrate the compass on a BerryIMUv1 or
#   BerryIMUv2.
#
#   Start this program and rotate your BerryIMU in all directions. 
#   You will see the maximum and minimum values change. 
#   After about 30secs or when the values are not changing, press Ctrl-C.
#   The program will printout some text which you then need to add to
#   berryIMU.py or berryIMU-simple.py
import math

import sys, signal, os
import time

from LSM6DS33 import LSM6DS33
import datetime


def handle_ctrl_c(signal, frame):
    gyrxavg = math.fsum(gyro_x) / len(gyro_x)
    gyryavg = math.fsum(gyro_y) / len(gyro_y)
    gyrzavg = math.fsum(gyro_z) / len(gyro_z)
    os.system("clear")
    print("X: {:5}".format(gyrxavg))
    print("Y: {:5}".format(gyryavg))
    print("Z: {:5}".format(gyrzavg))
    sys.exit(130)  # 130 is standard exit code for ctrl-c


gyroscope = LSM6DS33()

# This will capture exit when using Ctrl-C
signal.signal(signal.SIGINT, handle_ctrl_c)

a = datetime.datetime.now()

# Preload the variables used to keep track of the minimum and maximum values
gyro_x = set()
gyro_y = set()
gyro_z = set()
time_start = time.time()
while time.time() - time_start < 2:
    a = gyroscope.get_gyroscope_data()
    time.sleep(.05)

try:
    while True:

        gyr_data = gyroscope.get_gyroscope_data()
        gyro_x.add(gyr_data.x)
        gyro_y.add(gyr_data.y)
        gyro_z.add(gyr_data.z)
        gyrxavg = math.fsum(gyro_x) / len(gyro_x)
        gyryavg = math.fsum(gyro_y) / len(gyro_y)
        gyrzavg = math.fsum(gyro_z) / len(gyro_z)

        os.system("clear")
        print("X: {:5}".format(gyrxavg))
        print("Y: {:5}".format(gyryavg))
        print("Z: {:5}".format(gyrzavg))



        # slow program down a bit, makes the output more readable
        time.sleep(1/10)
except:
    os.system("clear")
    print("X: {:5}".format(gyrxavg))
    print("Y: {:5}".format(gyryavg))
    print("Z: {:5}".format(gyrzavg))