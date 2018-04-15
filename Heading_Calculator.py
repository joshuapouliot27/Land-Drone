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


import datetime
import math

from vectors import Vector

from LSM6DS33 import LSM6DS33
from LIS3MDL import LIS3MDL


class Heading_Calculator:
    def __init__(self, gyroscope_accelerometer: LSM6DS33, magnetometer: LIS3MDL):
        self.G_GAIN = 0.00875  # [deg/s/LSB]  If you change the dps for gyro, you need to update this value accordingly

        self.magXmax = 4233
        self.magYmax = 1944
        self.magZmax = 9243
        self.magXmin = -3091
        self.magYmin = -5348
        self.magZmin = 2001

        self.gyroXangle = 0.0
        self.gyroYangle = 0.0
        self.gyroZangle = 0.0

        self.magn = magnetometer
        self.gyro_accel = gyroscope_accelerometer
        self.olx_mag_value = Vector(0, 0, 0)
        self.old_acc_value = Vector(0, 0, 0)

        self.mag_lpf_factor = 0.4
        self.acc_lpf_factor = 0.1

        self.a = datetime.datetime.now()
        self.b = 0

    def calculate_tilt_compensated_heading(self):
        # Read the accelerometer,gyroscope and magnetometer values
        acc_data = self.gyro_accel.get_accelerometer_data()
        acc_data.x = acc_data.x * self.acc_lpf_factor + self.old_acc_value.x * (1 - self.acc_lpf_factor)
        acc_data.y = acc_data.y * self.acc_lpf_factor + self.old_acc_value.y * (1 - self.acc_lpf_factor)
        acc_data.z = acc_data.z * self.acc_lpf_factor + self.old_acc_value.z * (1 - self.acc_lpf_factor)
        gyro_data = self.gyro_accel.get_gyroscope_data()
        magn_data = self.magn.get_magnetometer_data()
        magn_data.x = magn_data.x * self.mag_lpf_factor + self.olx_mag_value.x * (1-self.mag_lpf_factor)
        magn_data.y = magn_data.y * self.mag_lpf_factor + self.olx_mag_value.y * (1 - self.mag_lpf_factor)
        magn_data.z = magn_data.z * self.mag_lpf_factor + self.olx_mag_value.z * (1 - self.mag_lpf_factor)
        self.old_acc_value = acc_data
        self.olx_mag_value = magn_data

        # Apply hard iron calibration to compass
        magn_data.x -= (self.magXmin + self.magXmax) / 2
        magn_data.y -= (self.magYmin + self.magYmax) / 2
        magn_data.z -= (self.magZmin + self.magZmax) / 2

        # Apply soft iron calibration
        magn_data.x = (magn_data.x - self.magXmin) / (self.magXmax - self.magXmin) * 2 - 1
        magn_data.y = (magn_data.y - self.magYmin) / (self.magYmax - self.magYmin) * 2 - 1
        magn_data.z = (magn_data.z - self.magZmin) / (self.magZmax - self.magZmin) * 2 - 1

        # Calculate loop Period(LP). How long between Gyro Reads
        self.b = datetime.datetime.now() - self.a
        self.a = datetime.datetime.now()
        LP = self.b.microseconds / (1000000 * 1.0)

        # Convert Gyro raw to degrees per second
        rate_gyr_x = gyro_data.x * self.G_GAIN
        rate_gyr_y = gyro_data.y * self.G_GAIN
        rate_gyr_z = gyro_data.z * self.G_GAIN

        # Calculate the angles from the gyro.
        self.gyroXangle += rate_gyr_x * LP
        self.gyroYangle += rate_gyr_y * LP
        self.gyroZangle += rate_gyr_z * LP

        # Convert Accelerometer values to degrees
        AccXangle = math.degrees(math.atan2(ACCy, ACCz) + math.pi)
        AccYangle = math.degrees(math.atan2(ACCz, ACCx) + math.pi)

        ####################################################################
        ######################Correct rotation value########################
        ####################################################################
        # Change the rotation value of the accelerometer to -/+ 180 and
        # move the Y axis '0' point to up.
        #
        # Two different pieces of code are used depending on how your IMU is mounted.
        # If IMU is up the correct way, Skull logo is facing down, Use these lines
        # AccXangle -= 180.0
        # if AccYangle > 90:
        #     AccYangle -= 270.0
        # else:
        #     AccYangle += 90.0
        # If IMU is upside down E.g Skull logo is facing up;
        if AccXangle >180:
            AccXangle -= 360.0
        AccYangle-=90
        if (AccYangle >180):
            AccYangle -= 360.0
        ############################ END ##################################

        ####################################################################
        ############################MAG direction ##########################
        ####################################################################
        # If IMU is upside down, then use this line.  It isnt needed if the
        # IMU is the correct way up
        magn_data.y *= -1
        #
        ############################ END ##################################

        # Calculate heading
        heading = 180 * math.atan2(MAGy, MAGx) / math.pi

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
        # pitch = math.asin(accXnorm)
        # roll = -math.asin(accYnorm / math.cos(pitch))
        #
        # Us these four lines when the IMU is upside down. Skull logo is facing up
        accXnorm = -accXnorm  # flip Xnorm as the IMU is upside down
        accYnorm = -accYnorm  # flip Ynorm as the IMU is upside down
        pitch = math.asin(accXnorm)
        roll = math.asin(accYnorm / math.cos(pitch))
        #
        ############################ END ##################################

        # Calculate the new tilt compensated values
        magnetometer_x_component = magn_data.x * math.cos(pitch) + magn_data.z * math.sin(pitch)
        magnetometer_y_component = magn_data.x * math.sin(roll) * math.sin(pitch) + magn_data.y * math.cos(roll) \
                                   - magn_data.z * math.sin(roll) * math.cos(pitch)

        # Calculate tilt compensated heading
        tilt_compensated_heading = 180 * math.atan2(magnetometer_y_component, magnetometer_x_component) / math.pi

        if tilt_compensated_heading < 0:
            tilt_compensated_heading += 360
        return tilt_compensated_heading
