import datetime
import math
import time

from headingcalculator import HeadingCalculator
from Background_Thread import Background_Thread
from imu_fusion import imu_fusion
from LIS3MDL import LIS3MDL
from LSM6DS33 import LSM6DS33
#import RTIMU

# setting_file = "./RTIMULib"
# rtimu = RTIMU.Settings(setting_file)
# imu = RTIMU.RTIMU(rtimu)
# if not imu.IMUInit():
#     print("IMU init failed!")
#     exit(1)


magn = LIS3MDL()
acc_gyr = LSM6DS33()
calc = HeadingCalculator(acc_gyr, magn)
points = set()
max_points = 5

# def imu_loop():
#    while True:
#        imu.IMURead()
#        time.sleep(poll_interval)


#imu.setSlerpPower(0.98)
#imu.setGyroEnable(True)
#imu.setAccelEnable(True)
#imu.setCompassEnable(True)
#poll_interval = imu.IMUGetPollInterval()
#thread = Background_Thread(imu_loop)

while True:
    # if len(points) >= max_points:
    #     for point in points:
    #         points.remove(point)
    #         break
    # points.add(calc.calculate_tilt_compensated_heading())
    # heading = math.fsum(points) / len(points)
    heading = calc.calculate_tilt_compensated_heading()
    print(time.strftime("%X") + ("; Heading: {:.5}".format(heading)))
    time.sleep(1 / 10)
