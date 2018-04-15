import datetime
import math
import time

from Heading_Calculator import Heading_Calculator
from LIS3MDL import LIS3MDL
from LSM6DS33 import LSM6DS33

magn = LIS3MDL()
acc_gyr = LSM6DS33()
calc = Heading_Calculator(acc_gyr, magn)
points = set()
max_points = 5
while True:
    if len(points) >= max_points:
        for point in points:
            points.remove(point)
            break
    points.add(calc.calculate_tilt_compensated_heading())
    heading = math.fsum(points) / len(points)
    print(time.strftime("%X") + "; Heading: "+str(heading))
    time.sleep(1/5)
