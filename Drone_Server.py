from multiprocessing import Process, Queue

import gpsd
import json
import logging
import math
import time

import RPi.GPIO as GPIO
from SimpleWebSocketServer import WebSocket, SimpleWebSocketServer
from websocket_server import WebsocketServer

from Background_Thread import Background_Thread
from Heading_Calculator import Heading_Calculator
from LIS3MDL import LIS3MDL
from LSM6DS33 import LSM6DS33

moving_left = False
moving_right = False
moving_forward = False
moving_backward = False

# Current Value Variables
current_latitude: float = None
current_longitude: float = None
current_direction_degrees: float = None
current_distance_ahead: float = None
dir_left = False
dir_right = False
dir_forward = False
dir_backward = False
current_pwm = 0

# Pin Number Variables
left_motor_direction_pin = 15
right_motor_direction_pin = 16
left_motor_pwm_speed_pin = 11
right_motor_pwm_speed_pin = 12
sonar_trig_pin = 18
sonar_echo_pin = 22

# GPIO variables
left_motor_pwm: GPIO.PWM = None
right_motor_pwm: GPIO.PWM = None

# IMU Variables
magnetometer: LIS3MDL = None
accelerometer_gyroscope: LSM6DS33 = None
heading_calculator: Heading_Calculator = None

# Frequency variables
main_loop_frequency = 25
# Misc Variables
trace = True
trace_loop = False
all_stop = False
max_pwm = 20000
max_turn_pwm = 8000
accelerometer_threshold = 0.05
accelerometer_offset_x = -0.007706830092610056
accelerometer_offset_y = -0.9543302538970905


class Websocket_Server(WebSocket):
    def handleMessage(self):
        json = web_socket_handler(self.data)
        if json is not None:
            self.sendMessage(json)


def web_socket_handler(message):
    if "return" in message:
        json_data = get_json_string()
        return json_data
    else:
        set_json_variables(message)
        return None


def get_json_string():
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


def set_json_variables(json_string):
    global moving_forward, moving_backward, moving_left, moving_right
    json_data = json.loads(json_string)
    moving_forward = bool(json_data["moving_forward"])
    moving_backward = bool(json_data["moving_backward"])
    moving_right = bool(json_data["moving_right"])
    moving_left = bool(json_data["moving_left"])


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


def setup():
    setup_logging()

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


def setup_logging():
    logging.basicConfig(format='%(asctime)s; %(levelname)s: %(message)s', datefmt='%m/%d/%Y %I:%M:%S %p',
                        filename="drone.log", level=logging.DEBUG)


def ramp_pwm(end):
    global current_pwm
    print("ramping pwm from "+str(current_pwm)+" to "+str(end))
    beginning = current_pwm
    if beginning is end:
        return
    step_max = 500
    step_freq = 20
    min_freq = 1000
    if beginning > end:
        steps = math.fabs((beginning-end) // step_max)
        left_over = math.fabs((beginning-end)) - steps * step_max
        for x in range(0, int(steps)):
            new_pwm = current_pwm - step_max
            set_pwm_freq(False, new_pwm)
            set_pwm_freq(True, new_pwm)
            current_pwm = new_pwm
            time.sleep(1 / step_freq)
        new_pwm = current_pwm - left_over
        set_pwm_freq(False, new_pwm)
        set_pwm_freq(True, new_pwm)
        current_pwm = new_pwm
        time.sleep(1 / step_freq)
        print("final pwm: "+str(new_pwm))
    else:
        steps = math.fabs((beginning - end) // step_max)
        left_over = math.fabs((beginning - end)) - steps * step_max
        for x in range(0, int(steps)):
            new_pwm = current_pwm + step_max
            set_pwm_freq(False, new_pwm)
            set_pwm_freq(True, new_pwm)
            current_pwm = new_pwm
            time.sleep(1/step_freq)
        new_pwm = current_pwm + left_over
        set_pwm_freq(False, new_pwm)
        set_pwm_freq(True, new_pwm)
        current_pwm = new_pwm
        time.sleep(1 / step_freq)
        print("final pwm: " + str(new_pwm))


def set_pwm_freq(is_left, freq):
    global current_pwm
    print("going from "+str(current_pwm)+" to "+str(freq)+" current_pwm > 0: "+str((current_pwm > 0))+" freq <= 0"+str(freq <= 0))
    if freq is current_pwm:
        return
    if is_left:
        if (freq <= 0) and (current_pwm > 0):
            left_motor_pwm.stop()
            current_pwm = 0
        elif 1000 <= freq <= 20000 and current_pwm > 0:
            left_motor_pwm.ChangeFrequency(freq)
            current_pwm = freq
        elif 1000 <= freq <= 20000 and current_pwm <= 0:
            left_motor_pwm.start(50)
            left_motor_pwm.ChangeFrequency(freq)
            current_pwm = freq
    else:
        if (freq <= 0) and (current_pwm > 0):
            right_motor_pwm.stop()
        elif 1000 <= freq <= 20000 and current_pwm > 0:
            right_motor_pwm.ChangeFrequency(freq)
        elif 1000 <= freq <= 20000 and current_pwm <= 0:
            right_motor_pwm.start(50)
            right_motor_pwm.ChangeFrequency(freq)


def set_motor_speed(percent, emergency=False):
    global current_left_pwm, current_right_pwm
    logging.info("Set motor speed to " + str(percent) + "%!")
    if emergency:
        if not dir_left or not dir_right:
            end_freq = percent * max_pwm
            set_pwm_freq(False, end_freq)
            set_pwm_freq(True, end_freq)
        else:
            end_freq = percent * max_turn_pwm
            set_pwm_freq(False, end_freq)
            set_pwm_freq(True, end_freq)
    else:
        if not dir_left or not dir_right:
            end_freq = percent * max_pwm
            ramp_pwm(end_freq)
        else:
            end_freq = percent * max_turn_pwm
            ramp_pwm(end_freq)


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
    if trace_loop:
        print("sonar loop")
    time_start = time.time()
    global current_distance_ahead
    current_distance_ahead = get_sonar_distance()


def gps_loop():
    if trace_loop:
        print("gps loop")
    time_start = time.time()
    get_position()


def imu_loop():
    if trace_loop:
        print("imu loop")
    time_start = time.time()
    get_true_heading()


def web_socket_loop():
    server = SimpleWebSocketServer('', 8081, Websocket_Server)
    server.serveforever()


def main_loop():
    while True:

        if trace_loop:
            print("Main loop")

        imu_loop()
        gps_loop()
        sonar_loop()

        # Distance Sensor
        if get_sonar_distance() <= 1 and current_pwm > 0:
            print("obstacle in the way, stopping")
            set_motor_speed(0, True)

        # if direction isn't proper, then stop moving change direction and start moving
        if not is_proper_direction():
            print("changing proper direction")
            if current_pwm > 0:
                set_motor_speed(0)
            set_proper_direction()
            # while not await check_constant_speed():
            # time.sleep(loop_Delay / 1000)
            set_motor_speed(1)
        # If distance is fine and remote button isn't pressed and not moving, then start moving
        if get_sonar_distance() > 1 and current_pwm <= 0 \
                and (moving_right or moving_left or moving_forward or moving_backward):
            print("started moving")
            set_motor_speed(1)

        # if not supposed to be moving, but is moving then stop moving
        if not moving_backward and not moving_forward and not moving_left and not moving_right and current_pwm > 0:
            print("stopping motion")
            set_motor_speed(0)
try:
    setup()
    print("Setup complete!")
    thread = Background_Thread(web_socket_loop)
    # web_socket_loop()
    main_loop()
except Exception as error:
    set_pwm_freq(False,0)
    set_pwm_freq(True,0)
    all_stop = True
    GPIO.cleanup()
    print("ERROR: " + str(error))
    print("cleaned up!")
