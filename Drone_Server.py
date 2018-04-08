import logging, json, time

import math, watchdog, gpsd

from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler, FileModifiedEvent

from Heading_Calculator import Heading_Calculator
from LIS3MDL import LIS3MDL
from LSM6DS33 import LSM6DS33

# import RPi.GPIO as GPIO # For the pi
from RPi import GPIO  # For editing

GPIO.VERBOSE = False  # For editing

# Action Variables
moving_Left = False
moving_Right = False
moving_Forward = False
moving_Backward = False

# Current Value Variables
current_Latitude = None
current_Longitude = None
current_Direction_Degrees = None
current_Distance_Ahead = None
dir_Left = False
dir_Right = False
dir_Forward = False
dir_Backward = False
is_Moving = False

# Pin Number Variables
left_motor_direction_pin = 15
right_motor_direction_pin = 16
left_motor_pwm_speed_pin = 11
right_motor_pwm_speed_pin = 12
# gps_rx_pin = 8
# gps_tx_pin = 10
sonar_trig_pin = 18
sonar_echo_pin = 22
stop_button_input_pin = 19

# GPIO variables
left_motor_pwm = None
right_motor_pwm = None

# File Variables
program_Changed_File = False
json_Filename = "land_drone.JSON"

# Misc Variables
max_pwm = 20000
max_turn_pwm = 8000
accel_threshold = 0.05
stop_Everything = False
loop_Delay = 1  # How much time in milliseconds to wait after every loop
accel_offset_x = -0.007706830092610056
accel_offset_y = -0.9543302538970905


class JSON_File_Handler(FileSystemEventHandler):
    def __init__(self, function, filename):
        self.filename = filename
        self.function = function
        super()

    def on_modified(self, event: FileModifiedEvent):
        if self.filename in event.src_path:
            self.function()


def construct_json_dictionary(moving_left, moving_right, moving_forward, moving_backword, current_latitude,
                              current_longitude, current_direction_degrees, current_distance_ahead, stop_everything):
    data = {
        "moving_left": moving_left,
        "moving_right": moving_right,
        "moving_forward": moving_forward,
        "moving_backward": moving_backword,
        "current_latitude": current_latitude,
        "current_longitude": current_longitude,
        "current_direction_degrees": current_direction_degrees,
        "current_distance_ahead": current_distance_ahead,
        "stop_everything": stop_everything
    }
    return data


def write_json_file(moving_left, moving_right, moving_forward, moving_backword, current_latitude, current_longitude,
                    current_direction_degrees, current_distance_ahead, stop_everything, json_filename):
    wrote_file = False
    data = construct_json_dictionary(moving_left, moving_right, moving_forward, moving_backword,
                                     current_latitude, current_longitude, current_direction_degrees,
                                     current_distance_ahead, stop_everything)

    try:
        with open(json_filename, 'w') as file:
            json.dump(data, file, indent=4, ensure_ascii=False, sort_keys=True)
            wrote_file = True
    except OSError:
        logging.error("Could not open " + str(json_filename) + ".")
    return wrote_file


def read_json_file(json_filename):
    data = None
    with open(json_filename, 'r') as file:
        data = json.load(file)
    return data


def set_variables_from_json_data():
    json_data = read_json_file(json_Filename)
    global moving_Forward, moving_Backward, moving_Left, moving_Right, stop_Everything
    moving_Forward = bool(json_data["moving_forward"])
    moving_Backward = bool(json_data["moving_backward"])
    moving_Right = bool(json_data["moving_right"])
    moving_Left = bool(json_data["moving_left"])
    stop_Everything = bool(json_data["stop_everything"])
    return


def get_position_and_direction():
    got_current_position = False
    got_direction = False
    global current_Latitude, current_Longitude, current_Direction_Degrees
    gps_packet = gpsd.get_current()
    if gps_packet.mode > 1:
        current_Longitude = gps_packet.lon
        current_Latitude = gps_packet.lat
        got_current_position = True
    return got_current_position


def get_distance_ahead():
    time_start = 0
    time_end = 0
    distance = None

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

    return distance


def setup_gpio_pins():
    gpio_pins_setup = False
    GPIO.setmode(GPIO.BOARD)
    # Stop Button
    GPIO.setup(stop_button_input_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    # GPS
    setup_gps()
    # IMU

    # Sonar
    GPIO.setup(sonar_echo_pin, GPIO.IN)
    GPIO.setup(sonar_trig_pin, GPIO.OUT)
    GPIO.output(sonar_trig_pin, False)
    # Drive Motor
    GPIO.setup(left_motor_direction_pin, GPIO.OUT)
    GPIO.output(left_motor_direction_pin, False)
    GPIO.setup(right_motor_direction_pin, GPIO.OUT)
    GPIO.output(right_motor_direction_pin, False)
    GPIO.setup(left_motor_pwm_speed_pin, GPIO.OUT)
    left_motor_pwm = GPIO.PWM(left_motor_pwm_speed_pin, 1)
    GPIO.setup(right_motor_direction_pin, GPIO.OUT)
    right_motor_pwm = GPIO.PWM(right_motor_pwm_speed_pin, 1)
    time.sleep(1)
    return gpio_pins_setup


def setup_gps():
    gpsd.connect()
    packet = gpsd.get_current()
    if packet.mode < 2:
        logging.warning("GPS does not have a fix!")
    counter = 0
    while packet.mode < 2:
        if counter > 150:
            logging.error("GPS cannot get a fix!")
            return False
        packet = gpsd.get_current()
        logging.warning("GPS still does not have a fix.")
        counter += 1
        time.sleep(.2)
    logging.debug("GPS has fix.")
    return True


def setupLogging():
    logging.basicConfig(format='%(asctime)s; %(levelname)s: %(message)s', datefmt='%m/%d/%Y %I:%M:%S %p',
                        filename="drone.log", level=logging.DEBUG)
    return


def check_stop_button():
    button_pressed = False
    if not (GPIO.input(stop_button_input_pin)):
        button_pressed = True
    return button_pressed


def setMotorSpeed(isLeft, perc):
    if isLeft:
        if perc is 0:
            left_motor_pwm.stop()
        elif perc > 0 and not is_Moving:
            left_motor_pwm.start(50)
        if dir_Left or dir_Right:
            left_motor_pwm.ChangeFrequency(perc * max_pwm)
        else:
            left_motor_pwm.ChangeFrequency(perc * max_turn_pwm)
    else:
        if perc is 0:
            right_motor_pwm.stop()
        elif perc > 0 and not is_Moving:
            right_motor_pwm.start(50)
        if dir_Left or dir_Right:
            left_motor_pwm.ChangeFrequency(perc * max_pwm)
        else:
            left_motor_pwm.ChangeFrequency(perc * max_turn_pwm)


def set_motor_direction(isLeft, forw):
    if isLeft:
        GPIO.output(left_motor_direction_pin, forw)
    else:
        GPIO.output(right_motor_direction_pin, forw)


def set_proper_direction():
    global dir_Left, dir_Backward, dir_Forward, dir_Right
    if moving_Forward and not dir_Forward:
        set_motor_direction(True, True)
        set_motor_direction(False, True)
        dir_Forward = True
        dir_Left = False
        dir_Right = False
        dir_Backward = False
    if moving_Left and not dir_Left:
        set_motor_direction(True, False)
        set_motor_direction(False, True)
        dir_Left = True
        dir_Forward = False
        dir_Right = False
        dir_Backward = False
    if moving_Right and not dir_Right:
        set_motor_direction(True, True)
        set_motor_direction(False, False)
        dir_Right = True
        dir_Left = False
        dir_Forward = False
        dir_Backward = False
    if moving_Backward and not dir_Backward:
        set_motor_direction(True, False)
        set_motor_direction(False, False)
        dir_Backward = True
        dir_Left = False
        dir_Right = False
        dir_Forward = False


def is_proper_direction():
    if moving_Forward and not dir_Forward:
        return False
    if moving_Left and not dir_Left:
        return False
    if moving_Right and not dir_Right:
        return False
    if moving_Backward and not dir_Backward:
        return False
    return True


def check_constant_speed():
    accel_data = accel_gyro.get_accelerometer_data()
    ACCx = accel_data.x
    ACCy = accel_data.y
    ACCz = accel_data.z
    accXnorm = (ACCx / math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)) + accel_offset_x
    accYnorm = (ACCy / math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)) + accel_offset_y
    if math.fabs(accXnorm) < accel_threshold and math.fabs(accYnorm) < accel_threshold:
        return True
    else:
        return False


magnetometer = LIS3MDL()
accel_gyro = LSM6DS33()
setupLogging()
setup_gpio_pins()
setup_gps()
get_position_and_direction()
set_motor_direction(True, True)
set_motor_direction(True, True)
heading_calculator = Heading_Calculator(accel_gyro, magnetometer)
dir_Forward = True
event_handler = JSON_File_Handler(set_variables_from_json_data, json_Filename)
observer = Observer()
observer.schedule(event_handler, path='./')
observer.start()
moving_Left = True

while True:

    # Remote Stop Button
    if stop_Everything and is_Moving:
        setMotorSpeed(True, 0)
        setMotorSpeed(False, 0)
        is_Moving = False

    # Distance Sensor
    if get_distance_ahead() <= 4 and is_Moving:
        setMotorSpeed(True, 0)
        setMotorSpeed(False, 0)
        is_Moving = False

    # if direction isn't proper, then stop moving change direction and start moving
    if not is_proper_direction():
        if is_Moving:
            setMotorSpeed(True, 0)
            setMotorSpeed(False, 0)
            is_Moving = False
        set_proper_direction()
        while not check_constant_speed():
            time.sleep(loop_Delay / 1000)
        setMotorSpeed(True, 1)
        setMotorSpeed(False, 1)
        is_Moving = True

    # If distance is fine and remote button isn't pressed and not moving, then start moving
    if get_distance_ahead() > 4 and not is_Moving and not stop_Everything \
            and (moving_Right or moving_Left or moving_Forward or moving_Backward):
        setMotorSpeed(True, 1)
        setMotorSpeed(False, 1)
        is_Moving = True

    # if not supposed to be moving, but is moving then stop moving
    if not moving_Backward and not moving_Forward and not moving_Left and not moving_Right and is_Moving:
        setMotorSpeed(True, 0)
        setMotorSpeed(False, 0)
        is_Moving = False

    time.sleep(loop_Delay / 1000)
    break

time.sleep(5)
left_motor_pwm.stop()
right_motor_pwm.stop()
observer.stop()
observer.join()
GPIO.cleanup()
