import logging, json, time
import watchdog

# import RPi.GPIO as GPIO # For the pi
from RPi import GPIO # For editing
GPIO.VERBOSE = False # For editing
import gpsd
GPIO.VERBOSE = False

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

# Pin Number Variables
left_motor_direction_pin = 15
right_motor_direction_pin = 16
left_motor_pwm_speed_pin = 11
right_motor_speed_pin = 12
#gps_rx_pin = 8
#gps_tx_pin = 10
sonar_trig_pin = 18
sonar_echo_pin = 22
stop_button_input_pin = 19

# File Variables
program_Changed_File = False
json_Filename = "land_drone_JSON_file.JSON"

# Misc Variables
stop_Everything = False
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
    global moving_Forward, moving_Backward, moving_Left, moving_Right, stop_Everything
    moving_Forward = bool(json_data["moving_forward"])
    moving_Backward = bool(json_data["moving_backward"])
    moving_Right = bool(json_data["moving_right"])
    moving_Left = bool(json_data["moving_left"])
    return

def get_position_and_direction():
    got_current_position = False
    global current_Latitude, current_Longitude, current_Direction_Degrees
    gps_packet = gpsd.get_current()
    print("Latitude: "+str(gps_packet.lat)+" Longitude: "+str(gps_packet.lon))
    return got_current_position

def get_distance_ahead():
    time_start, time_end = 0
    distance = None

    GPIO.output(sonar_trig_pin,False)
    time.sleep(.05)

    GPIO.output(sonar_trig_pin,True)
    time.sleep(.000001)
    GPIO.output(sonar_trig_pin,False)

    while (GPIO.input(sonar_echo_pin) == 0):
        time_start = time.time()
    while (GPIO.input(sonar_echo_pin) == 1):
        time_end = time.time()

    total_time = time_end - time_start
    distance = (total_time / 2) * 1125.33 # Calculated in ft/s

    return distance

def setup_gpio_pins():
    gpio_pins_setup = False
    GPIO.setmode(GPIO.BOARD)
    # Stop Button
    GPIO.setup(stop_button_input_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    # GPS
    # Sonar
    GPIO.setup(sonar_echo_pin,GPIO.IN)
    GPIO.setup(sonar_trig_pin,GPIO.OUT)
    GPIO.output(sonar_trig_pin,False)
    # Drive Motor
    GPIO.setup(left_motor_direction_pin,GPIO.OUT)
    GPIO.output(left_motor_direction_pin,False)
    GPIO.setup(right_motor_direction_pin,GPIO.OUT)
    GPIO.output(right_motor_direction_pin, False)
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
        counter+=1
        time.sleep(.2)
    logging.debug("GPS has fix.")
    return True

def setupLogging():
    logging.basicConfig(format='%(asctime)s; %(levelname)s: %(message)s', datefmt='%m/%d/%Y %I:%M:%S %p',filename="drone.log",level=logging.DEBUG)
    return

def check_stop_button():
    button_pressed = False
    if (not (GPIO.input(stop_button_input_pin))):
        button_pressed = True
    return button_pressed

print("Wrote File: " + str(write_json_file(moving_Left, moving_Right, moving_Forward, moving_Backward, current_Latitude,
                                           current_Longitude, current_Direction_Degrees, current_Distance_Ahead,
                                           stop_Everything, json_Filename)) + ".")
print("Read File as: " + str(read_json_file(json_Filename)) + ".")
print("What None prints as in JSON: " + str(json.dumps({"a": None})))

# while True:
#
#     if (stop_Everything):
#         break
#
#     time.sleep((loop_Delay) / 1000)
#     break
#GPIO.cleanup()

setupLogging()
setup_gpio_pins()
setup_GPS()
current_Distance_Ahead = get_distance_ahead()
print("Distance: " + str(current_Distance_Ahead) + "ft")
get_position_and_direction()
GPIO.cleanup()
