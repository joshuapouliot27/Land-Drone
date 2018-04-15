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


class HeadingCalculator:
    def __init__(self, gyroscope_accelerometer: LSM6DS33, magnetometer: LIS3MDL):

        self.magXmax = 4233
        self.magYmax = 1944
        self.magZmax = 9243
        self.magXmin = -3091
        self.magYmin = -5348
        self.magZmin = 2001

        self.magn = magnetometer
        self.gyro_accel = gyroscope_accelerometer
        self.olx_mag_value = Vector(0, 0, 0)
        self.old_acc_value = Vector(0, 0, 0)

        self.mag_lpf_factor = 0.4
        self.acc_lpf_factor = 0.1


    def calculate_tilt_compensated_heading(self):
        # Read the accelerometer,gyroscope and magnetometer values
        acc_data = self.gyro_accel.get_accelerometer_data()
        #acc_data.x = acc_data.x * self.acc_lpf_factor + self.old_acc_value.x * (1 - self.acc_lpf_factor)
        #acc_data.y = acc_data.y * self.acc_lpf_factor + self.old_acc_value.y * (1 - self.acc_lpf_factor)
        #acc_data.z = acc_data.z * self.acc_lpf_factor + self.old_acc_value.z * (1 - self.acc_lpf_factor)
        magn_data = self.magn.get_magnetometer_data()
        #magn_data.x = magn_data.x * self.mag_lpf_factor + self.olx_mag_value.x * (1 - self.mag_lpf_factor)
        #magn_data.y = magn_data.y * self.mag_lpf_factor + self.olx_mag_value.y * (1 - self.mag_lpf_factor)
        #magn_data.z = magn_data.z * self.mag_lpf_factor + self.olx_mag_value.z * (1 - self.mag_lpf_factor)
        self.old_acc_value = acc_data
        self.olx_mag_value = magn_data

        magn_x_offset = (self.magXmin + self.magXmax) / 2
        magn_y_offset = (self.magYmin + self.magYmax) / 2
        magn_z_offset = (self.magZmin + self.magZmax) / 2
        magn_avg_offset = (magn_z_offset + magn_y_offset + magn_x_offset) / 3
        magn_scale_x = magn_avg_offset / magn_x_offset
        magn_scale_y = magn_avg_offset / magn_y_offset
        magn_scale_z = magn_avg_offset / magn_z_offset

        # Apply calibration to compass
        magn_data.x = (magn_data.x - magn_x_offset) * magn_scale_x
        magn_data.y = (magn_data.y - magn_y_offset) * magn_scale_y
        magn_data.z = (magn_data.z - magn_z_offset) * magn_scale_z

        # Convert Accelerometer values to degrees
        AccXangle = math.degrees(math.atan2(acc_data.y, acc_data.z) + math.pi)
        AccYangle = math.degrees(math.atan2(acc_data.z, acc_data.x) + math.pi)

        if AccXangle >180:
            AccXangle -= 360.0

        if AccYangle >180:
            AccYangle -= 360.0

        ####################################################################
        ############################MAG direction ##########################
        ####################################################################
        # If IMU is upside down, then use this line.  It isnt needed if the
        # IMU is the correct way up
        # magn_data.y *= -1
        #
        ############################ END ##################################

        # Calculate heading
        heading = math.degrees(math.atan2(magn_data.y, magn_data.x))

        # Only have our heading between 0 and 360
        if heading < 0:
            heading += 360

        # Normalize accelerometer raw values.
        accXnorm = acc_data.x / math.sqrt(acc_data.x ** 2 + acc_data.y ** 2 + acc_data.z ** 2)
        accYnorm = acc_data.y / math.sqrt(acc_data.x ** 2 + acc_data.y ** 2 + acc_data.z ** 2)

        ####################################################################
        ###################Calculate pitch and roll#########################
        ####################################################################
        # Us these two lines when the IMU is up the right way. Skull logo is facing down
        pitch = math.asin(accXnorm)
        roll = -math.asin(accYnorm / math.cos(pitch))
        #
        # Us these four lines when the IMU is upside down. Skull logo is facing up
        # accXnorm = -accXnorm  # flip Xnorm as the IMU is upside down
        # accYnorm = -accYnorm  # flip Ynorm as the IMU is upside down
        # pitch = math.asin(accXnorm)
        # roll = math.asin(accYnorm / math.cos(pitch))
        #
        ############################ END ##################################

        # Calculate the new tilt compensated values
        magnetometer_x_component = magn_data.x * math.cos(pitch) + magn_data.z * math.sin(pitch)
        magnetometer_y_component = magn_data.x * math.sin(roll) * math.sin(pitch) + magn_data.y * math.cos(roll) \
                                   - magn_data.z * math.sin(roll) * math.cos(pitch)

        # Calculate tilt compensated heading
        tilt_compensated_heading =  math.degrees(math.atan2(magnetometer_y_component, magnetometer_x_component))
        if tilt_compensated_heading < 0:
            tilt_compensated_heading += 360
        return tilt_compensated_heading
