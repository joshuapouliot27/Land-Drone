import asyncio
import threading

import gpsd
import json
import logging
import math
import time

import RPi.GPIO as GPIO
from websocket_server import WebsocketServer

from Heading_Calculator import Heading_Calculator
from LIS3MDL import LIS3MDL
from LSM6DS33 import LSM6DS33

# Action Variables
moving_left = False
moving_right = False
moving_forward = False
moving_backward = False

# Current Value Variables
current_latitude = None
current_longitude = None
current_direction_degrees = None
current_distance_ahead = None
dir_left = False
dir_right = False
dir_forward = False
dir_backward = False
is_moving = False

# Pin Number Variables
left_motor_direction_pin = 15
right_motor_direction_pin = 16
left_motor_pwm_speed_pin = 11
right_motor_pwm_speed_pin = 12
sonar_trig_pin = 18
sonar_echo_pin = 22

# GPIO variables
left_motor_pwm = None
right_motor_pwm = None

# IMU Variables
magnetometer = None
accelerometer_gyroscope = None
heading_calculator = None

# Frequency variables
main_loop_frequency = 1
gps_frequency = 20
imu_frequency = 50
sonar_frequency = 20

# Misc Variables
all_stop = False
max_pwm = 20000
max_turn_pwm = 8000
accelerometer_threshold = 0.05
accelerometer_offset_x = -0.007706830092610056
accelerometer_offset_y = -0.9543302538970905


async def web_socket_handler(client, server, message):
    if "return" in message:
        json_data = await get_json_string()
        server.send(json_data)
    else:
        set_json_variables(message)


async def get_json_string():
    data = {
        "moving_left": moving_left,
        "moving_right": moving_right,
        "moving_forward": moving_forward,
        "moving_backward": moving_backward,
        "current_latitude": current_latitude,
        "current_longitude": current_longitude,
        "current_direction_degrees": current_direction_degrees,
        "current_distance_ahead": current_distance_ahead,
    }
    return json.dumps(data)


async def set_json_variables(json_string):
    global moving_forward, moving_backward, moving_left, moving_right
    json_data = json.loads(json_string)
    moving_forward = bool(json_data["moving_forward"])
    moving_backward = bool(json_data["moving_backward"])
    moving_right = bool(json_data["moving_right"])
    moving_left = bool(json_data["moving_left"])
    return


def get_position():
    global current_latitude, current_longitude, current_direction_degrees
    gps_packet = gpsd.get_current()
    if gps_packet.mode > 1:
        current_longitude = gps_packet.lon
        current_latitude = gps_packet.lat
    logging.debug("Current Position: Latitude: {0:.2}; Longitude: {1:.2}".format(current_latitude, current_longitude))


def get_sonar_distance():
    time_start = 0
    time_end = 0

    GPIO.output(sonar_trig_pin, False)
    time.sleep(.05)

    GPIO.output(sonar_trig_pin, True)
    time.sleep(.000001)
    GPIO.output(sonar_trig_pin, False)

    while GPIO.input(sonar_echo_pin) == 0:
        time_start = time.time()
    while GPIO.input(sonar_echo_pin) == 1:
        time_end = time.time()

    total_time = time_end - time_start
    distance = (total_time / 2) * 1125.33  # Calculated in ft/s
    logging.debug("Sonar distance: {0:.2} ft".format(distance))
    return distance


def setup_imu():
    global magnetometer, accelerometer_gyroscope, heading_calculator
    magnetometer = LIS3MDL()
    accelerometer_gyroscope = LSM6DS33()
    heading_calculator = Heading_Calculator(accelerometer_gyroscope, magnetometer)
    logging.info("IMU setup!")


def setup_sonar():
    GPIO.setup(sonar_echo_pin, GPIO.IN)
    GPIO.setup(sonar_trig_pin, GPIO.OUT)
    GPIO.output(sonar_trig_pin, False)
    logging.info("Sonar setup!")


def setup_motor_drivers():
    global left_motor_pwm, right_motor_pwm
    # Left
    GPIO.setup(left_motor_direction_pin, GPIO.OUT)
    GPIO.output(left_motor_direction_pin, False)
    GPIO.setup(right_motor_direction_pin, GPIO.OUT)
    GPIO.output(right_motor_direction_pin, False)
    GPIO.setup(left_motor_pwm_speed_pin, GPIO.OUT)
    left_motor_pwm = GPIO.PWM(left_motor_pwm_speed_pin, 1)

    # Right
    GPIO.output(left_motor_direction_pin, False)
    GPIO.setup(right_motor_direction_pin, GPIO.OUT)
    GPIO.output(right_motor_direction_pin, False)
    GPIO.setup(right_motor_pwm_speed_pin, GPIO.OUT)
    right_motor_pwm = GPIO.PWM(right_motor_pwm_speed_pin, 1)

    set_motor_direction(True, True)
    set_motor_direction(True, True)

    logging.info("Motor drivers setup!")


def setup_web_socket_server():
    server = WebsocketServer(8081)
    server.set_fn_message_received(web_socket_handler)
    server.run_forever()


def setup():
    setupLogging()

    GPIO.setmode(GPIO.BOARD)

    # GPS
    setup_gps()

    # IMU
    setup_imu()

    # Sonar
    setup_sonar()

    # Motor Drivers
    setup_motor_drivers()

    logging.info("Setup complete!")


def setup_gps():
    gpsd.connect()
    packet = gpsd.get_current()
    if packet.mode < 2:
        logging.warning("GPS does not have a fix!")
    counter = 0
    while packet.mode < 2:
        if counter > 150:
            logging.error("GPS cannot get a fix!")
            return
        packet = gpsd.get_current()
        logging.warning("GPS still does not have a fix.")
        counter += 1
        time.sleep(.2)
    logging.info("GPS has fix.")


def setupLogging():
    logging.basicConfig(format='%(asctime)s; %(levelname)s: %(message)s', datefmt='%m/%d/%Y %I:%M:%S %p',
                        filename="drone.log", level=logging.INFO)


def set_motor_speed(is_left, percent):
    logging.info("Set motor speed to " + str(percent) + "%!")
    if is_left:
        if percent is 0 and is_moving:
            left_motor_pwm.stop()
        elif percent > 0 and not is_moving:
            left_motor_pwm.start(50)

        if (not dir_left or not dir_right) and percent > 0:
            left_motor_pwm.ChangeFrequency(percent * max_pwm)
        elif percent > 0:
            left_motor_pwm.ChangeFrequency(percent * max_turn_pwm)
    else:
        if percent is 0 and is_moving:
            right_motor_pwm.stop()
        elif percent > 0 and not is_moving:
            right_motor_pwm.start(50)

        if (not dir_left or not dir_right) and percent > 0:
            right_motor_pwm.ChangeFrequency(percent * max_pwm)
        elif percent > 0:
            right_motor_pwm.ChangeFrequency(percent * max_turn_pwm)


def set_motor_direction(is_left, forward):
    if is_left:
        GPIO.output(left_motor_direction_pin, forward)
    else:
        GPIO.output(right_motor_direction_pin, not forward)


def set_proper_direction():
    global dir_left, dir_backward, dir_forward, dir_right
    if moving_forward and not dir_forward:
        logging.info("Going forward!")
        set_motor_direction(True, True)
        set_motor_direction(False, True)
        dir_forward = True
        dir_left = False
        dir_right = False
        dir_backward = False
    if moving_left and not dir_left:
        logging.info("Going left!")
        set_motor_direction(True, False)
        set_motor_direction(False, True)
        dir_left = True
        dir_forward = False
        dir_right = False
        dir_backward = False
    if moving_right and not dir_right:
        logging.info("Going right!")
        set_motor_direction(True, True)
        set_motor_direction(False, False)
        dir_right = True
        dir_left = False
        dir_forward = False
        dir_backward = False
    if moving_backward and not dir_backward:
        logging.info("Going backward!")
        set_motor_direction(True, False)
        set_motor_direction(False, False)
        dir_backward = True
        dir_left = False
        dir_right = False
        dir_forward = False


def is_proper_direction():
    if moving_forward and not dir_forward:
        return False
    if moving_left and not dir_left:
        return False
    if moving_right and not dir_right:
        return False
    if moving_backward and not dir_backward:
        return False
    return True


def check_constant_speed():
    accel_data = accelerometer_gyroscope.get_accelerometer_data()
    ACCx = accel_data.x
    ACCy = accel_data.y
    ACCz = accel_data.z
    accXnorm = (ACCx / math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)) + accelerometer_offset_x
    accYnorm = (ACCy / math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)) + accelerometer_offset_y
    if math.fabs(accXnorm) < accelerometer_threshold and math.fabs(accYnorm) < accelerometer_threshold:
        return True
    else:
        return False


def only_positive_numbers(number: float):
    if number > 0:
        return number
    else:
        return 0


def get_true_heading():
    global current_direction_degrees
    current_direction_degrees = heading_calculator.calculate_tilt_compensated_heading()


def sonar_loop():
    while True:
        if all_stop:
            break
        time_start = time.time()
        global current_distance_ahead
        current_distance_ahead = get_sonar_distance()
        time.sleep(only_positive_numbers((1 / sonar_frequency) - (time.time() - time_start)))


def gps_loop():
    while True:
        if all_stop:
            break
        time_start = time.time()
        get_position()
        time.sleep(only_positive_numbers((1 / gps_frequency) - (time.time() - time_start)))


def imu_loop():
    while True:
        if all_stop:
            break
        time_start = time.time()
        get_true_heading()
        time.sleep(only_positive_numbers((1 / imu_frequency) - (time.time() - time_start)))


def web_socket_loop():
    setup_web_socket_server()


def main_loop():
    while True:
        if all_stop:
            break
        global is_moving

        time_start = time.time()

        # Distance Sensor
        if get_sonar_distance() <= 4 and is_moving:
            set_motor_speed(True, 0)
            set_motor_speed(False, 0)
            is_moving = False

        # if direction isn't proper, then stop moving change direction and start moving
        if not is_proper_direction():
            if is_moving:
                set_motor_speed(True, 0)
                set_motor_speed(False, 0)
                is_moving = False
            set_proper_direction()
            # while not await check_constant_speed():
            # time.sleep(loop_Delay / 1000)
            set_motor_speed(True, 1)
            set_motor_speed(False, 1)
            is_moving = True
        print("can move: " + str(get_sonar_distance() > 4 and not is_moving \
                                       and (moving_right or moving_left or moving_forward or moving_backward)))
        # If distance is fine and remote button isn't pressed and not moving, then start moving
        if get_sonar_distance() > 4 and not is_moving \
                and (moving_right or moving_left or moving_forward or moving_backward):
            set_motor_speed(True, 1)
            set_motor_speed(False, 1)
            is_moving = True

        # if not supposed to be moving, but is moving then stop moving
        if not moving_backward and not moving_forward and not moving_left and not moving_right and is_moving:
            set_motor_speed(True, 0)
            set_motor_speed(False, 0)
            is_moving = False

        time.sleep(only_positive_numbers((1 / main_loop_frequency) - (time.time() - time_start)))

setup()
print("Setup complete!")
try:
    threads = set()
    threads.add(threading.Thread(target=gps_loop))
    threads.add(threading.Thread(target=sonar_loop))
    threads.add(threading.Thread(target=imu_loop))
    threads.add(threading.Thread(target=main_loop))
    threads.add(threading.Thread(target=web_socket_loop))
    for thread in threads:
        thread.start()
        thread.join()
except:
    all_stop = True
    asyncio.get_event_loop().stop()
    GPIO.cleanup()
