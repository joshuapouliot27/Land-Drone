import datetime
import math
import time

from headingcalculator import HeadingCalculator
from Background_Thread import Background_Thread
from imu_fusion import imu_fusion
from LIS3MDL import LIS3MDL
from LSM6DS33 import LSM6DS33

magn = LIS3MDL()
acc_gyr = LSM6DS33()
calc = HeadingCalculator(acc_gyr, magn)
# fus = imu_fusion(acc_gyr, magn)
points = set()
max_points = 5
# thread = Background_Thread(fus.loop)
while True:
    if len(points) >= max_points:
        for point in points:
            points.remove(point)
            break
    points.add(calc.calculate_tilt_compensated_heading())
    # points.add(fus.get_true_heading())
    heading = math.fsum(points) / len(points)
    print(time.strftime("%X") + "; Heading: " + str(heading))
    time.sleep(1 / 2)
