import time

from Heading_Calculator import Heading_Calculator
from LIS3MDL import LIS3MDL
from LSM6DS33 import LSM6DS33

magn = LIS3MDL()
acc_gyr = LSM6DS33()
calc = Heading_Calculator(acc_gyr, magn)
while True:
    heading = calc.calculate_tilt_compensated_heading()
    print("Heading: "+str(heading))
    time.sleep(1/10)