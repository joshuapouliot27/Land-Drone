#!/usr/bin/python
#   This program is used to calibrate the compass on a BerryIMUv1 or
#   BerryIMUv2.
#
#   Start this program and rotate your BerryIMU in all directions. 
#   You will see the maximum and minimum values change. 
#   After about 30secs or when the values are not changing, press Ctrl-C.
#   The program will printout some text which you then need to add to
#   berryIMU.py or berryIMU-simple.py


import sys, signal, os
import time
import math

from LIS3MDL import LIS3MDL
import datetime


def handle_ctrl_c(signal, frame):
    os.system("clear")
    print("magXmin = ", magXmin)
    print("magYmin = ", magYmin)
    print("magZmin = ", magZmin)
    print("magXmax = ", magXmax)
    print("magYmax = ", magYmax)
    print("magZmax = ", magZmax)
    sys.exit(130)  # 130 is standard exit code for ctrl-c


magnetometer = LIS3MDL()

# This will capture exit when using Ctrl-C
signal.signal(signal.SIGINT, handle_ctrl_c)

a = datetime.datetime.now()

# Preload the variables used to keep track of the minimum and maximum values
magXmin = 9999999
magYmin = 9999999
magZmin = 9999999
magXmax = -9999999
magYmax = -9999999
magZmax = -9999999
try:
    while True:

    # Read magnetometer values
        MAGx = magnetometer.get_magnetometer_data().x
        MAGy = magnetometer.get_magnetometer_data().y
        MAGz = magnetometer.get_magnetometer_data().z

        if MAGx > magXmax:
            magXmax = MAGx
        if MAGy > magYmax:
            magYmax = MAGy
        if MAGz > magZmax:
            magZmax = MAGz

        if MAGx < magXmin:
            magXmin = MAGx
        if MAGy < magYmin:
            magYmin = MAGy
        if MAGz < magZmin:
            magZmin = MAGz

        os.system("clear")
        print("X: {:5} to {:5}".format(magXmin, magXmax))
        print("Y: {:5} to {:5}".format(magYmin, magYmax))
        print("Z: {:5} to {:5}".format(magZmin, magZmax))



        # slow program down a bit, makes the output more readable
        time.sleep(1/10)
except:
    os.system("clear")
    print("X: {:5} to {:5}".format(magXmin, magXmax))
    print("Y: {:5} to {:5}".format(magYmin, magYmax))
    print("Z: {:5} to {:5}".format(magZmin, magZmax))