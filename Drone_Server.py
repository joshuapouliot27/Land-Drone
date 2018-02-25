import logging, json, time
import watchdog

# import RPi.GPIO as GPIO
# import fakeRPiGPIO

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
left_motor_direction_pin = 0
right_motor_direction_pin = 0
motor_pwm_speed_pin = 0
gps_1_pin = 0
gps_2_pin = 0
gps_3_pin = 0
gps_4_pin = 0
sonar_1_pin = 0
sonar_2_pin = 0
stop_button_output_pin = 0
stop_button_input_pin = 0

# File Variables
program_Changed_File = False
json_Filename = "land_drone_JSON_file"

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
    #do stuff
    return got_current_position

def get_distance_ahead():
    got_distance_ahead = False
    global current_Distance_Ahead
    #do stuff
    return got_distance_ahead

def setup_gpio_pine():
    gpio_pins_setup = False
    #do stuff
    return gpio_pins_setup

print("Wrote File: " + str(write_json_file(moving_Left, moving_Right, moving_Forward, moving_Backward, current_Latitude,
                                           current_Longitude, current_Direction_Degrees, current_Distance_Ahead,
                                           stop_Everything, json_Filename)) + ".")
print("Read File as: " + str(read_json_file(json_Filename)) + ".")
print("What None prints as in JSON: " + str(json.dumps({"a": None})))

while True:
    if (stop_Everything):
        break

    time.sleep((loop_Delay) / 1000)
    break
