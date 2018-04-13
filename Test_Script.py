import logging, json, time

import RPi.GPIO as GPIO  # For the pi
# from RPi import GPIO # For editing
# GPIO.VERBOSE = False # For editing
import gpsd

# Action Variables
moving_left = False
moving_right = False
moving_forward = False
moving_backward = False

# Current Value Variables
current_latitude = None
current_longitude = None
current_direction_degrees = None
current_Distance_Ahead = None

# Pin Number Variables
left_motor_direction_pin = 22
right_motor_direction_pin = 23
left_motor_pwm_speed_pin = 17
right_motor_pwm_speed_pin = 18
sonar_trig_pin = 24
sonar_echo_pin = 25
stop_button_input_pin = 10

# GPIO variables
left_motor_pwm = None
right_motor_pwm = None

# File Variables
program_Changed_File = False
json_Filename = "land_drone_JSON_file.JSON"

# Misc Variables
max_pwm = 20000
stop_everything = False
loop_Delay = 1  # How much time in milliseconds to wait after every loop


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


def set_variables_from_json_data(json_data):
    global moving_forward, moving_backward, moving_left, moving_right, stop_everything
    moving_Forward = bool(json_data["moving_forward"])
    moving_Backward = bool(json_data["moving_backward"])
    moving_Right = bool(json_data["moving_right"])
    moving_Left = bool(json_data["moving_left"])
    return


def get_position_and_direction():
    got_current_position = False
    global current_latitude, current_longitude, current_direction_degrees
    gps_packet = gpsd.get_current()
    print("Latitude: " + str(gps_packet.lat) + " Longitude: " + str(gps_packet.lon))
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

    while (GPIO.input(sonar_echo_pin) == 0):
        time_start = time.time()
    while (GPIO.input(sonar_echo_pin) == 1):
        time_end = time.time()

    total_time = time_end - time_start
    distance = (total_time / 2) * 1125.33  # Calculated in ft/s

    return distance


def setup_gpio_pins():
    global left_motor_pwm, right_motor_pwm
    gpio_pins_setup = False
    GPIO.setmode(GPIO.BCM)
    # Stop Button
    GPIO.setup(stop_button_input_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    # GPS
    # Sonar
    GPIO.setup(sonar_echo_pin, GPIO.IN)
    GPIO.setup(sonar_trig_pin, GPIO.OUT)
    GPIO.output(sonar_trig_pin, False)
    # Drive Motor Direction
    GPIO.setup(left_motor_direction_pin, GPIO.OUT)
    GPIO.output(left_motor_direction_pin, False)

    GPIO.setup(right_motor_direction_pin, GPIO.OUT)
    GPIO.output(right_motor_direction_pin, False)
    # Driver motor PWM
    GPIO.setup(left_motor_pwm_speed_pin, GPIO.OUT)
    left_motor_pwm = GPIO.PWM(left_motor_pwm_speed_pin, 1)

    GPIO.setup(right_motor_pwm_speed_pin, GPIO.OUT)
    right_motor_pwm = GPIO.PWM(right_motor_pwm_speed_pin, 1)
    time.sleep(1)
    return gpio_pins_setup


def setup_GPS():
    gpsd.connect()
    packet = gpsd.get_current()
    if (packet.mode < 2):
        logging.warning("GPS does not have a fix!")
    counter = 0
    while (packet.mode < 2):
        if (counter > (150)):
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
    if (not (GPIO.input(stop_button_input_pin))):
        button_pressed = True
    return button_pressed


def setMotorSpeed(isLeft, hz):
    if (isLeft):
        left_motor_pwm.ChangeFrequency(hz)
    else:
        right_motor_pwm.ChangeFrequency(hz)

    return


def setMotorDirection(isLeft, forw):
    if (isLeft):
        GPIO.output(left_motor_direction_pin, forw)
    else:
        GPIO.output(right_motor_direction_pin, not forw)


setup_gpio_pins()
setupLogging()
setMotorDirection(True, True)
setMotorDirection(False, True)
isStopped = True
isForward = True
left_motor_pwm.stop()
right_motor_pwm.stop()

while True:

    driver_motor_hz = int(input("Enter Hz to driver motor, '-2' to reverse direction, or '-1' to quit: "))
    if driver_motor_hz is -1:
        break
    elif driver_motor_hz is 0:
        left_motor_pwm.stop()
        right_motor_pwm.stop()
        isStopped = True
    elif driver_motor_hz is -2:
        if isForward:
            setMotorDirection(True, False)
            setMotorDirection(False, False)
            isForward = False
            print("MOTORS GOING FORWARD")
        else:
            setMotorDirection(True, True)
            setMotorDirection(False, True)
            isForward = True
            print("MOTORS GOING BACKWARD")
    elif driver_motor_hz < 0 or driver_motor_hz > 20000:
        print("The Hz (entered: " + str(driver_motor_hz) + ") must be a number between 0-20000!")
    else:
        if isStopped:
            left_motor_pwm.start(50)
            right_motor_pwm.start(50)
            print("STARTED MOTORS!")
            isStopped = False
        setMotorSpeed(True, driver_motor_hz)
        setMotorSpeed(False, driver_motor_hz)
        print("driver motor at " + str(driver_motor_hz) + "Hz.")

    time.sleep(loop_Delay / 1000)

if left_motor_pwm is not None:
    left_motor_pwm.stop()
if right_motor_pwm is not None:
    right_motor_pwm.stop()
GPIO.cleanup()
