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
from LIS3MDL import LIS3MDL
from LSM6DS33 import LSM6DS33
import datetime


class Heading_Calculator:
    def __init__(self, gyroscope_accelerometer, magnetometer):
        self.RAD_TO_DEG = 57.29578
        self.M_PI = 3.14159265358979323846
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

    async def readACCx(self):
        return self.gyro_accel.get_accelerometer_data().x

    async def readACCy(self):
        return self.gyro_accel.get_accelerometer_data().y

    async def readACCz(self):
        return self.gyro_accel.get_accelerometer_data().z

    async def readMAGx(self):
        return self.magn.get_magnetometer_data().x

    async def readMAGy(self):
        return self.magn.get_magnetometer_data().y

    async def readMAGz(self):
        return self.magn.get_magnetometer_data().z

    async def readGYRx(self):
        return self.gyro_accel.get_gyroscope_data().x

    async def readGYRy(self):
        return self.gyro_accel.get_gyroscope_data().y

    async def readGYRz(self):
        return self.gyro_accel.get_gyroscope_data().z

    async def calculate_tilt_compensated_heading(self):
        # Read the accelerometer,gyroscope and magnetometer values
        ACCx = await self.readACCx()
        ACCy = await self.readACCy()
        ACCz = await self.readACCz()
        GYRx = await self.readGYRx()
        GYRy = await self.readGYRy()
        GYRz = await self.readGYRz()
        MAGx = await self.readMAGx()
        MAGy = await self.readMAGy()
        MAGz = await self.readMAGz()

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
        AccXangle = (math.atan2(ACCy, ACCz) + self.M_PI) * self.RAD_TO_DEG
        AccYangle = (math.atan2(ACCz, ACCx) + self.M_PI) * self.RAD_TO_DEG

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
        heading = 180 * math.atan2(MAGy, MAGx) / self.M_PI

        # Only have our heading between 0 and 360
        if heading < 0:
            heading += 360

        # Normalize accelerometer raw values.
        accXnorm = ACCx / math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)
        accYnorm = ACCy / math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)

        ####################################################################
        ###################Calculate pitch and roll#########################
        ####################################################################
        # Us these two lines when the IMU is up the right way. Skull logo is facing down
        pitch = math.asin(accXnorm)
        roll = -math.asin(accYnorm / math.cos(pitch))
        #
        # Us these four lines when the IMU is upside down. Skull logo is facing up
        # accXnorm = -accXnorm				#flip Xnorm as the IMU is upside down
        # accYnorm = -accYnorm				#flip Ynorm as the IMU is upside down
        # pitch = math.asin(accXnorm)
        # roll = math.asin(accYnorm/math.cos(pitch))
        #
        ############################ END ##################################

        # Calculate the new tilt compensated values
        magXcomp = MAGx * math.cos(pitch) + MAGz * math.sin(pitch)
        magYcomp = MAGx * math.sin(roll) * math.sin(pitch) + MAGy * math.cos(roll) - MAGz * math.sin(roll) * math.cos(
            pitch)

        # Calculate tilt compensated heading
        tiltCompensatedHeading = 180 * math.atan2(magYcomp, magXcomp) / self.M_PI

        if tiltCompensatedHeading < 0:
            tiltCompensatedHeading += 360
        return tiltCompensatedHeading
