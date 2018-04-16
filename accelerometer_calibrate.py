#!/usr/bin/python
#   This program is used to calibrate the compass on a BerryIMUv1 or
#   BerryIMUv2.
#
#   Start this program and rotate your BerryIMU in all directions. 
#   You will see the maximum and minimum values change. 
#   After about 30secs or when the values are not changing, press Ctrl-C.
#   The program will printout some text which you then need to add to
#   berryIMU.py or berryIMU-simple.py


import sys, os
import time

from LSM6DS33 import LSM6DS33
import datetime


def handle_ctrl_c(signal, frame):
    os.system("clear")
    print("accXmin = ", accXmin)
    print("accYmin = ", accYmin)
    print("accZmin = ", accZmin)
    print("accXmax = ", accXmax)
    print("accYmax = ", accYmax)
    print("accZmax = ", accZmax)
    sys.exit(130)  # 130 is standard exit code for ctrl-c


accelerometer = LSM6DS33()

a = datetime.datetime.now()

# Preload the variables used to keep track of the minimum and maximum values
accXmin = 9999999
accYmin = 9999999
accZmin = 9999999
accXmax = -9999999
accYmax = -9999999
accZmax = -9999999
try:
    while True:

    # Read acc values
        acc_data = accelerometer.get_accelerometer_data()
        accx = acc_data.x
        accy = acc_data.y
        accz = acc_data.z

        if accx > accXmax:
            accXmax = accx
        if accy > accYmax:
            accYmax = accy
        if accz > accZmax:
            accZmax = accz

        if accx < accXmin:
            accXmin = accx
        if accy < accYmin:
            accYmin = accy
        if accz < accZmin:
            accZmin = accz

        os.system("clear")
        print("X: {:5} to {:5}".format(accXmin, accXmax))
        print("Y: {:5} to {:5}".format(accYmin, accYmax))
        print("Z: {:5} to {:5}".format(accZmin, accZmax))



        # slow program down a bit, makes the output more readable
        time.sleep(1/10)
except:
    os.system("clear")
    print("X: {:5} to {:5}".format(accXmin, accXmax))
    print("Y: {:5} to {:5}".format(accYmin, accYmax))
    print("Z: {:5} to {:5}".format(accZmin, accZmax))