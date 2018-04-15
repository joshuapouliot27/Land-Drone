#!/usr/bin/python
#
#	This program  reads the angles from the acceleromter, gyrscope
#	and mangnetometeron a BerryIMU connected to a Raspberry Pi.
#
#	This program includes a number of calculations to improve the
#	values returned from BerryIMU. If this is new to you, it
#	may be worthwhile first to look at berryIMU-simple.py, which
#	has a much more simplified version of code which would be easier
#	to read.
#
#
#	http://ozzmaker.com/
#
#    Copyright (C) 2016  Mark Williams
#    This library is free software; you can redistribute it and/or
#    modify it under the terms of the GNU Library General Public
#    License as published by the Free Software Foundation; either
#    version 2 of the License, or (at your option) any later version.
#    This library is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
#    Library General Public License for more details.
#    You should have received a copy of the GNU Library General Public
#    License along with this library; if not, write to the Free
#    Software Foundation, Inc., 59 Temple Place - Suite 330, Boston,
#    MA 02111-1307, USA


import os
import time
import math

from numpy.core.umath import degrees

from LIS3MDL import LIS3MDL
from LSM6DS33 import LSM6DS33
import datetime
from sympy import cos, sin, atan2, sqrt, asin, pi


class Heading_Calculator:
    def __init__(self, gyroscope_accelerometer, magnetometer):
        self.RAD_TO_DEG = 57.29578
        self.G_GAIN = 0.00875  # [deg/s/LSB]  If you change the dps for gyro, you need to update this value accordingly

        self.magXmax = 4149
        self.magYmax = 1643
        self.magZmax = 8910
        self.magXmin = -2745
        self.magYmin = -4985
        self.magZmin = 2674

        self.gyroXangle = 0.0
        self.gyroYangle = 0.0
        self.gyroZangle = 0.0

        self.magn = magnetometer
        self.gyro_accel = gyroscope_accelerometer

        self.a = datetime.datetime.now()
        self.b = 0

    def readACCx(self):
        return self.gyro_accel.get_accelerometer_data().x

    def readACCy(self):
        return self.gyro_accel.get_accelerometer_data().y

    def readACCz(self):
        return self.gyro_accel.get_accelerometer_data().z

    def readMAGx(self):
        return self.magn.get_magnetometer_data().x

    def readMAGy(self):
        return self.magn.get_magnetometer_data().y

    def readMAGz(self):
        return self.magn.get_magnetometer_data().z

    def readGYRx(self):
        return self.gyro_accel.get_gyroscope_data().x

    def readGYRy(self):
        return self.gyro_accel.get_gyroscope_data().y

    def readGYRz(self):
        return self.gyro_accel.get_gyroscope_data().z

    def calculate_tilt_compensated_heading(self):
        # Read the accelerometer,gyroscope and magnetometer values
        ACCx = self.readACCx()
        ACCy = self.readACCy()
        ACCz = self.readACCz()
        GYRx = self.readGYRx()
        GYRy = self.readGYRy()
        GYRz = self.readGYRz()
        MAGx = self.readMAGx()
        MAGy = self.readMAGy()
        MAGz = self.readMAGz()

        # Apply hard iron calibration to compass
        MAGx -= (self.magXmin + self.magXmax) / 2
        MAGy -= (self.magYmin + self.magYmax) / 2
        MAGz -= (self.magZmin + self.magZmax) / 2

        # Calculate loop Period(LP). How long between Gyro Reads
        self.b = datetime.datetime.now() - self.a
        self.a = datetime.datetime.now()
        LP = self.b.microseconds / (1000000 * 1.0)

        # Convert Gyro raw to degrees per second
        rate_gyr_x = GYRx * self.G_GAIN
        rate_gyr_y = GYRy * self.G_GAIN
        rate_gyr_z = GYRz * self.G_GAIN

        # Calculate the angles from the gyro.
        self.gyroXangle += rate_gyr_x * LP
        self.gyroYangle += rate_gyr_y * LP
        self.gyroZangle += rate_gyr_z * LP

        # Convert Accelerometer values to degrees
        AccXangle = degrees(atan2(ACCy, ACCz) + pi)
        AccYangle = degrees(atan2(ACCz, ACCx) + pi)

        ####################################################################
        ######################Correct rotation value########################
        ####################################################################
        # Change the rotation value of the accelerometer to -/+ 180 and
        # move the Y axis '0' point to up.
        #
        # Two different pieces of code are used depending on how your IMU is mounted.
        # If IMU is up the correct way, Skull logo is facing down, Use these lines
        AccXangle -= 180.0
        if AccYangle > 90:
            AccYangle -= 270.0
        else:
            AccYangle += 90.0
        #
        ############################ END ##################################

        # Calculate heading
        heading = 180 * atan2(MAGy, MAGx) / pi

        # Only have our heading between 0 and 360
        if heading < 0:
            heading += 360

        # Normalize accelerometer raw values.
        accXnorm = ACCx / sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)
        accYnorm = ACCy / sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)

        ####################################################################
        ###################Calculate pitch and roll#########################
        ####################################################################
        # Us these two lines when the IMU is up the right way. Skull logo is facing down
        pitch = asin(accXnorm)
        roll = -asin(accYnorm / math.cos(pitch))
        #
        # Us these four lines when the IMU is upside down. Skull logo is facing up
        # accXnorm = -accXnorm				#flip Xnorm as the IMU is upside down
        # accYnorm = -accYnorm				#flip Ynorm as the IMU is upside down
        # pitch = asin(accXnorm)
        # roll = asin(accYnorm/math.cos(pitch))
        #
        ############################ END ##################################

        # Calculate the new tilt compensated values
        magnetometer_x_component = MAGx * cos(pitch) + MAGz * sin(pitch)
        magnetometer_y_component = MAGx * sin(roll) * sin(pitch) + MAGy * cos(roll) - MAGz * sin(roll) * cos(pitch)

        # Calculate tilt compensated heading
        tilt_compensated_heading = 180 * atan2(magnetometer_y_component, magnetometer_x_component) / pi

        if tilt_compensated_heading < 0:
            tilt_compensated_heading += 360
        return tilt_compensated_heading
