import math, time
from LSM6DS33 import LSM6DS33

accXnorm = 0
accYnorm = 0
accXnorms = 0
accYnorms = 0
total_nums = 0
LSM6DS33 = LSM6DS33()

seconds = 10
loop_delay = 1  # in Hz


def calc_norm_accel():
    accel_data = LSM6DS33.get_accelerometer_data()
    ACCx = accel_data.x
    ACCy = accel_data.y
    ACCz = accel_data.z
    accXnorm = ACCx / math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)
    accYnorm = ACCy / math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)


time_start = time.time()

for x in range(0, int(seconds / loop_delay)):
    calc_norm_accel()
    accXnorms += accXnorm
    accYnorms += accYnorm
    total_nums += 1

seconds_took = time.time() - time_start

avgx = accXnorms / total_nums
avgy = accYnorms / total_nums
print("In " + str(seconds_took) + " seconds.  avg x: " + str(avgx) + ", avg y:" + str(avgy))
