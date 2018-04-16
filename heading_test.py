import datetime
import math
import time

from headingcalculator import HeadingCalculator
from Background_Thread import Background_Thread
from imu_fusion import imu_fusion
from LIS3MDL import LIS3MDL
from LSM6DS33 import LSM6DS33
import RTIMU

setting_file = "./RTIMULib"
rtimu = RTIMU.Settings(setting_file)
imu = RTIMU.RTIMU(rtimu)
if not imu.IMUInit():
    print("IMU init failed!")
    exit(1)
# magn = LIS3MDL()
# acc_gyr = LSM6DS33()
# calc = HeadingCalculator(acc_gyr, magn)
# fus = imu_fusion(acc_gyr, magn)
# points = set()
# max_points = 5
# thread = Background_Thread(fus.loop)
imu.setSlerpPower(0.02)
imu.setGyroEnable(True)
imu.setAccelEnable(True)
imu.setCompassEnable(True)
poll_interval = imu.IMUGetPollInterval()
while True:
    # if len(points) >= max_points:
    #     for point in points:
    #         points.remove(point)
    #         break
    # points.add(calc.calculate_tilt_compensated_heading())
    # points.add(fus.get_true_heading())
    # heading = math.fsum(points) / len(points)
    data = imu.getIMUData()
    fusionPose = data["fusionPose"]
    roll_rad = fusionPose[0]
    pitch_rad = fusionPose[1]
    yaw_rad = fusionPose[2]
    x = math.cos(yaw_rad) * math.cos(pitch_rad)
    y = math.sin(yaw_rad) * math.cos(pitch_rad)
    heading = math.degrees(math.atan2(y, x))
    print(time.strftime("%X") + "; Heading: " + str(heading))
    time.sleep(1 / 2)
