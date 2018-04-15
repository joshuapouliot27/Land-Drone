import Math

import logging

from smbus2 import SMBus
from vectors import Vector


class LSM6DS33:
    possible_i2c_addresses = [0x6b]

    # IDs
    FUNC_CFG_ACCESS = 0x01

    FIFO_CTRL1 = 0x06
    FIFO_CTRL2 = 0x07
    FIFO_CTRL3 = 0x08
    FIFO_CTRL4 = 0x09
    FIFO_CTRL5 = 0x0A
    ORIENT_CFG_G = 0x0B

    INT1_CTRL = 0x0D
    INT2_CTRL = 0x0E
    WHO_AM_I = 0x0F
    CTRL1_XL = 0x10
    CTRL2_G = 0x11
    CTRL3_C = 0x12
    CTRL4_C = 0x13
    CTRL5_C = 0x14
    CTRL6_C = 0x15
    CTRL7_G = 0x16
    CTRL8_XL = 0x17
    CTRL9_XL = 0x18
    CTRL10_C = 0x19

    WAKE_UP_SRC = 0x1B
    TAP_SRC = 0x1C
    D6D_SRC = 0x1D
    STATUS_REG = 0x1E

    OUT_TEMP_L = 0x20
    OUT_TEMP_H = 0x21
    OUTX_L_G = 0x22
    OUTX_H_G = 0x23
    OUTY_L_G = 0x24
    OUTY_H_G = 0x25
    OUTZ_L_G = 0x26
    OUTZ_H_G = 0x27
    OUTX_L_XL = 0x28
    OUTX_H_XL = 0x29
    OUTY_L_XL = 0x2A
    OUTY_H_XL = 0x2B
    OUTZ_L_XL = 0x2C
    OUTZ_H_XL = 0x2D

    FIFO_STATUS1 = 0x3A
    FIFO_STATUS2 = 0x3B
    FIFO_STATUS3 = 0x3C
    FIFO_STATUS4 = 0x3D
    FIFO_DATA_OUT_L = 0x3E
    FIFO_DATA_OUT_H = 0x3F
    TIMESTAMP0_REG = 0x40
    TIMESTAMP1_REG = 0x41
    TIMESTAMP2_REG = 0x42

    STEP_TIMESTAMP_L = 0x49
    STEP_TIMESTAMP_H = 0x4A
    STEP_COUNTER_L = 0x4B
    STEP_COUNTER_H = 0x4C

    FUNC_SRC = 0x53

    TAP_CFG = 0x58
    TAP_THS_6D = 0x59
    INT_DUR2 = 0x5A
    WAKE_UP_THS = 0x5B
    WAKE_UP_DUR = 0x5C
    FREE_FALL = 0x5D
    MD1_CFG = 0x5E
    MD2_CFG = 0x5F

    # Values
    WHO_ID = 0x69
    HIGH_PERFORMANCE_MODE = 0x80

    def __init__(self):
        self.i2c_bus = SMBus(1)
        self.logger = logging.getLogger()
        self.address = None
        if not self.find_i2c_address():
            self.logger.error("Could not connect to LSM6DS33!")
        else:
            self.set_default_settings()

    def find_i2c_address(self):
        for address in self.possible_i2c_addresses:
            self.logger.debug("Trying address: " + str(address))
            try:
                if self.i2c_bus.read_byte_data(address, self.WHO_AM_I) == self.WHO_ID:
                    self.address = address
                    self.logger.debug("Found LSM6DS33 at "+str(self.address))
                    return True
            except:
                self.logger.debug("LSM6DS33 not found at "+str(address))
        self.logger.debug("LSM6DS33 not found from any addresses!")
        return False

    def write_data(self, register, data):
        try:
            self.i2c_bus.write_byte_data(self.address, register, data)
            return True
        except:
            self.logger.error("Could not write " + str(data) + " to " + str(register)
                              + " at " + str(self.address) + ".")
            return False

    def read_data(self, register):
        try:
            return self.i2c_bus.read_byte_data(self.address, register)
        except:
            self.logger.error("Could not read from " + str(register) + " at " + str(self.address) + ".")
            return None

    def set_default_settings(self):
        if self.i2c_bus is not None:
            # Set accelerometer to high performance
            if not self.write_data(self.CTRL1_XL, self.HIGH_PERFORMANCE_MODE):
                self.logger.error("Could not set accelerometer to high performance mode!")
            # Set gyroscope to high performance
            if not self.write_data(self.CTRL2_G, self.HIGH_PERFORMANCE_MODE):
                self.logger.error("Could not set gyroscope to high performance mode!")

    def get_gyroscope_data(self):
        vector = None
        gyro_x = Math.two_bytes_to_number(self.read_data(self.OUTX_H_G),
                                          self.read_data(self.OUTX_L_G))
        gyro_y = Math.two_bytes_to_number(self.read_data(self.OUTY_H_G),
                                          self.read_data(self.OUTY_L_G))
        gyro_z = Math.two_bytes_to_number(self.read_data(self.OUTZ_H_G),
                                          self.read_data(self.OUTZ_L_G))
        vector = Vector(gyro_x, gyro_y, gyro_z)
        return vector

    def get_accelerometer_data(self):
        vector = None
        accel_x = Math.two_bytes_to_number(self.read_data(self.OUTX_H_XL),
                                           self.read_data(self.OUTX_L_XL))
        accel_y = Math.two_bytes_to_number(self.read_data(self.OUTY_H_XL),
                                           self.read_data(self.OUTY_L_XL))
        accel_z = Math.two_bytes_to_number(self.read_data(self.OUTZ_H_XL),
                                           self.read_data(self.OUTZ_L_XL))
        vector = Vector(accel_x, accel_y, accel_z)
        return vector
