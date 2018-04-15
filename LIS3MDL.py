import Math

import logging

from smbus2 import SMBus
from vectors import Vector


class LIS3MDL:
    possible_i2c_addresses = [0x1e]

    # IDs
    WHO_AM_I = 0x0F

    CTRL_REG1 = 0x20
    CTRL_REG2 = 0x21
    CTRL_REG3 = 0x22
    CTRL_REG4 = 0x23
    CTRL_REG5 = 0x24

    STATUS_REG = 0x27
    OUT_X_L = 0x28
    OUT_X_H = 0x29
    OUT_Y_L = 0x2A
    OUT_Y_H = 0x2B
    OUT_Z_L = 0x2C
    OUT_Z_H = 0x2D
    TEMP_OUT_L = 0x2E
    TEMP_OUT_H = 0x2F
    INT_CFG = 0x30
    INT_SRC = 0x31
    INT_THS_L = 0x32
    INT_THS_H = 0x33

    # Values
    WHO_ID = 0x3D
    HIGH_PERFORMANCE_MODE_XY = 0x70
    HIGH_PERFORMANCE_MODE_Z = 0x0C
    CONTINUOUS_CONVERSION_MODE = 0x00
    FOUR_GAUSS_SCALE_MODE = 0x00

    def __init__(self):
        self.i2c_bus = SMBus(1)
        self.logger = logging.getLogger()
        self.address = None
        if not self.find_i2c_address():
            self.logger.error("Could not connect to LIS3MDL!")
        else:
            self.set_default_settings()

    def find_i2c_address(self):
        for address in self.possible_i2c_addresses:
            self.logger.debug("Trying address: " + str(address))
            try:
                if self.i2c_bus.read_byte_data(address, self.WHO_AM_I) == self.WHO_ID:
                    self.address = address
                    self.logger.debug("Found LIS3MDL at "+str(self.address))
                    return True
            except:
                self.logger.debug("LIS3MDL not found at "+str(address))
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
            # Set X and Y to high performance
            if not self.write_data(self.CTRL_REG1, self.HIGH_PERFORMANCE_MODE_XY):
                self.logger.error("Could not set X and Y to high performance mode!")
            # Set Z to high performance
            if not self.write_data(self.CTRL_REG4, self.HIGH_PERFORMANCE_MODE_Z):
                self.logger.error("Could not set Z to high performance mode!")
            if not self.write_data(self.CTRL_REG2, self.FOUR_GAUSS_SCALE_MODE):
                self.logger.error("Could not set to +/- 4 gauss scale mode!")
            if not self.write_data(self.CTRL_REG3, self.CONTINUOUS_CONVERSION_MODE):
                self.logger.error("Could not set to continuous conversion mode!")

    def get_magnetometer_data(self):
        vector = None
        mag_x = Math.two_bytes_to_number(self.read_data(self.OUT_X_H),
                                         self.read_data(self.OUT_X_L))
        mag_y = Math.two_bytes_to_number(self.read_data(self.OUT_Y_H),
                                         self.read_data(self.OUT_Y_L))
        mag_z = Math.two_bytes_to_number(self.read_data(self.OUT_Z_H),
                                         self.read_data(self.OUT_Z_L))
        vector = Vector(mag_x, mag_y, mag_z)
        return vector
