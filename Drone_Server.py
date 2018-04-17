from multiprocessing import Process, Queue
from typing import List

import gpsd
import json
import logging
import math
import time
import Math

import RPi.GPIO as GPIO
from SimpleWebSocketServer import WebSocket, SimpleWebSocketServer
from websocket_server import WebsocketServer

from Background_Thread import Background_Thread
# from headingcalculator import HeadingCalculator

# from LIS3MDL import LIS3MDL
# from LSM6DS33 import LSM6DS33

moving_left = False
moving_right = False
moving_forward = False
moving_backward = False

# Current Value Variables
current_latitude: float = None
current_longitude: float = None
current_direction_degrees: float = None
current_distance_ahead: float = 0
dir_left = False
dir_right = False
dir_forward = False
dir_backward = False
stop_everything = False
gps_lat_points = set()
gps_lon_points = set()
direction_points = set()
imu_points = set()
sonar_points = set()
current_pwm: List[int] = [0, 0]

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

# # IMU Variables
# magnetometer: LIS3MDL = None
# accelerometer_gyroscope: LSM6DS33 = None
# heading_calculator: HeadingCalculator = None

# Frequency variables
main_loop_frequency = 5000
# imu_frequency = 5000
gps_frequency = 5000
sonar_frequency = 5000

# Averaging variables
sonar_points_num_averaging = 5
gps_points_num_averaging = 5
# imu_points_num_averaging = 5

# Misc Variables
trace = True
trace_loop = False
all_stop = False
max_left_pwm = 20000
max_right_pwm = 20000
max_left_turn_pwm = 8000
max_right_turn_pwm = 8000
less_turn_percent = 0.2
accelerometer_threshold = 0.05
accelerometer_offset_x = -0.007706830092610056
accelerometer_offset_y = -0.9543302538970905

# Automated Variables
automated_mode = False
was_automated = False
sonar_min_distance = 1
gps_target = [0, 0]
direction_target = 0
gps_tolerance = 4  # in meters
direction_tolerance = 5  # in degrees
current_gps_index = -1
gps_points = [
    [44.9063300, -68.6683193],
    [44.9063243, -68.6682348],
    [44.9062683, -68.6681932],
    [44.9062256, -68.6680524],
    [44.9062740, -68.6679786],
    [44.9063576, -68.6679491],
    [44.9068411, -68.6680645],
    [44.9069075, -68.6681315],
    [44.9069505, -68.6683270],
    [44.9069161, -68.6687002],
    [44.9068230, -68.6688034],
    [44.9066948, -68.6688343],
    [44.9062085, -68.6687592],
    [44.9061610, -68.6657135],
    [44.9061610, -68.6686090],
    [44.9062142, -68.6685312],
    [44.9062977, -68.6684856],
    [44.9063224, -68.6683971]
]  # [[lat, lon], [lat, lon]...]
finished = False


class web_socket_Server(WebSocket):
    def handle_message(self):
        json_input = web_socket_handler(self.data)
        if json_input is not None:
            self.sendMessage(json_input)


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
        "stop_everything": stop_everything,
        "automated": automated_mode
    }
    return json.dumps(data)


def set_json_variables(json_string):
    global moving_forward, moving_backward, moving_left, moving_right, stop_everything, automated_mode
    json_data = json.loads(json_string)
    moving_forward = bool(json_data["moving_forward"])
    moving_backward = bool(json_data["moving_backward"])
    moving_right = bool(json_data["moving_right"])
    moving_left = bool(json_data["moving_left"])
    stop_everything = bool(json_data["stop_everything"])
    automated_mode = bool(json_data["automated"])


def get_position():
    global current_latitude, current_longitude, current_direction_degrees
    gps_packet = gpsd.get_current()
    if gps_packet.mode > 1:
        if len(gps_lat_points) >= gps_points_num_averaging:
            for point in gps_lat_points:
                gps_lat_points.remove(point)
                break
        if len(gps_lon_points) >= gps_points_num_averaging:
            for point in gps_lon_points:
                gps_lon_points.remove(point)
                break
        if len(direction_points) >= gps_points_num_averaging:
            for point in direction_points:
                direction_points.remove(point)
                break
        gps_lat_points.add(gps_packet.lat)
        gps_lon_points.add(gps_packet.lon)
        direction_points.add(gps_packet.track)
        current_direction_degrees = math.fsum(direction_points) / len(direction_points)
        current_longitude = math.fsum(gps_lon_points) / len(gps_lon_points)
        current_latitude = math.fsum(gps_lat_points) / len(gps_lat_points)
    logging.debug("Current Position: Latitude: {0:.2}; Longitude: {1:.2}; direction: {2:.2}"
                  .format(current_latitude, current_longitude, current_direction_degrees))


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


# def setup_imu():
#     global magnetometer, accelerometer_gyroscope, heading_calculator
#     magnetometer = LIS3MDL()
#     accelerometer_gyroscope = LSM6DS33()
#     heading_calculator = HeadingCalculator(accelerometer_gyroscope, magnetometer)
#     logging.info("IMU setup!")


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

    # # IMU
    # setup_imu()

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


def ramp_pwm(end, isLeft):
    global current_pwm
    if isLeft:
        beginning = current_pwm[0]
    else:
        beginning = current_pwm[1]
    if beginning is end:
        return
    step_max = 1000
    step_freq = 1 / (step_max / 10000)
    if beginning > end:
        steps = math.fabs((beginning - end) // step_max)
        left_over = math.fabs((beginning - end)) - steps * step_max
        for x in range(0, int(steps)):
            if isLeft:
                new_pwm = current_pwm[0] - step_max
            else:
                new_pwm = current_pwm[1] - step_max
            set_pwm_freq(isLeft, new_pwm)
            if isLeft:
                current_pwm[0] = [new_pwm, new_pwm]
            else:
                current_pwm[1] = [new_pwm, new_pwm]
            time.sleep(1 / step_freq)
        if isLeft:
            new_pwm = current_pwm[0] - left_over
        else:
            new_pwm = current_pwm[1] - left_over
        set_pwm_freq(isLeft, new_pwm)
        if isLeft:
            current_pwm[0] = new_pwm
        else:
            current_pwm[1] = new_pwm
        time.sleep(1 / step_freq)
        print("final pwm: " + str(new_pwm))
    else:
        steps = math.fabs((beginning - end) // step_max)
        left_over = math.fabs((beginning - end)) - steps * step_max
        for x in range(0, int(steps)):
            if isLeft:
                new_pwm = current_pwm[0] + step_max
            else:
                new_pwm = current_pwm[1] + step_max
            set_pwm_freq(isLeft, new_pwm)
            if isLeft:
                current_pwm[0] = new_pwm
            else:
                current_pwm[1] = new_pwm
            time.sleep(1 / step_freq)
        if isLeft:
            new_pwm = current_pwm[0] + left_over
        else:
            new_pwm = current_pwm[1] + left_over
        set_pwm_freq(isLeft, new_pwm)
        if isLeft:
            current_pwm[0] = new_pwm
        else:
            current_pwm[1] = new_pwm
        time.sleep(1 / step_freq)
        print("final pwm: " + str(new_pwm))


def set_pwm_freq(is_left, freq):
    global current_pwm
    if freq is current_pwm:
        return
    if is_left:
        if (freq <= 0) and (is_moving()):
            left_motor_pwm.stop()
            current_pwm[0] = 0
        elif 500 <= freq <= 20000 and current_pwm > 0:
            left_motor_pwm.ChangeFrequency(freq)
            current_pwm[0] = freq
        elif 500 <= freq <= 20000 and current_pwm <= 0:
            left_motor_pwm.start(50)
            left_motor_pwm.ChangeFrequency(freq)
            current_pwm[0] = freq
    else:
        if (freq <= 0) and (is_moving()):
            right_motor_pwm.stop()
            current_pwm[1] = 0
        elif 500 <= freq <= 20000 and current_pwm > 0:
            right_motor_pwm.ChangeFrequency(freq)
            current_pwm[1] = freq
        elif 500 <= freq <= 20000 and current_pwm <= 0:
            right_motor_pwm.start(50)
            right_motor_pwm.ChangeFrequency(freq)
            current_pwm[1] = freq


def set_motor_speed(percent, emergency=False, is_left=None):
    logging.info("Set motor speed to " + str(percent) + "%!")
    if emergency and is_left is None:
        if not dir_left and not dir_right:
            set_pwm_freq(False, percent * max_right_pwm)
            set_pwm_freq(True, percent * max_left_pwm)
        else:
            set_pwm_freq(False, percent * max_right_turn_pwm)
            set_pwm_freq(True, percent * max_left_turn_pwm)
    elif is_left is None:
        if not dir_left and not dir_right:
            thread = Background_Thread(ramp_pwm, (percent * max_left_pwm, True))
            thread2 = Background_Thread(ramp_pwm, (percent * max_right_pwm, False))
        else:
            thread = Background_Thread(ramp_pwm, (percent * max_left_turn_pwm, True))
            thread2 = Background_Thread(ramp_pwm, (percent * max_right_turn_pwm, False))
    else:
        if is_left:
            end_freq = percent * max_left_pwm
        else:
            end_freq = percent * max_right_pwm
        thread = Background_Thread(ramp_pwm, (end_freq, is_left))


def set_motor_direction(is_left, forward):
    if is_left:
        GPIO.output(left_motor_direction_pin, not forward)
    else:
        GPIO.output(right_motor_direction_pin, forward)


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


def only_positive_numbers(number: float):
    if number > 0:
        return number
    else:
        return 0


# def get_true_heading():
#     global current_direction_degrees
#     if len(imu_points) >= imu_points_num_averaging:
#         for point in imu_points:
#             imu_points.remove(point)
#             break
#     imu_points.add(HeadingCalculator.calculate_tilt_compensated_heading())
#     current_direction_degrees = math.fsum(imu_points) / len(imu_points)


def sonar_loop():
    while True:
        if all_stop:
            break
        if trace_loop:
            print("sonar loop")
        global current_distance_ahead
        if len(sonar_points) >= sonar_points_num_averaging:
            for point in sonar_points:
                sonar_points.remove(point)
                break
        sonar_points.add(get_sonar_distance())
        current_distance_ahead = math.fsum(sonar_points) / len(sonar_points)
        time.sleep(1 / sonar_frequency)


def gps_loop():
    while True:
        if all_stop:
            break
        if trace_loop:
            print("gps loop")
        get_position()
        time.sleep(1 / gps_frequency)


# def imu_loop():
#     while True:
#         if all_stop:
#             break
#         if trace_loop:
#             print("imu loop")
#         get_true_heading()
#         time.sleep(1 / imu_frequency)


def web_socket_loop():
    server = SimpleWebSocketServer('', 8081, web_socket_Server)
    server.serveforever()


def correct_automated_direction():
    if math.fabs(current_direction_degrees - direction_target) < direction_tolerance:
        return True
    else:
        return False


def pos_degree(number):
    if number >= 360:
        new_num = number - 360
    elif number < 0:
        new_num = number + 360
    else:
        new_num = number
    return new_num


def should_turn_left():
    right_turn_degrees = pos_degree(direction_target - current_direction_degrees)
    left_turn_degrees = pos_degree(360 - right_turn_degrees)
    if left_turn_degrees <= right_turn_degrees:
        return True
    else:
        return False


def is_moving():
    if current_pwm[0] > 0 or current_pwm[1] > 0:
        return True
    else:
        return False


def reset_gps_target():
    global current_gps_index
    current_gps_index = -1


def get_next_gps_target():
    global current_gps_index
    current_gps_index += 1
    if current_gps_index >= len(gps_points):
        return None
    else:
        return gps_points[current_gps_index]


def main_loop():
    global was_automated, automated_mode, gps_target, direction_target, finished
    while True:

        if trace_loop:
            print("Main loop")
        if all_stop:
            break

        #       imu_loop()
        #       gps_loop()
        #       sonar_loop()

        # Distance Sensor
        if (stop_everything or current_distance_ahead <= sonar_min_distance) and current_pwm > 0:
            print("obstacle in the way or stop pressed, emergency stopping")
            set_motor_speed(0, True)
        if not automated_mode:
            if was_automated:
                reset_gps_target()
                was_automated = False
                set_motor_speed(0)
                set_motor_direction(True, True)
                set_motor_direction(False, True)
                finished = False
            # if direction isn't proper, then stop moving change direction and start moving
            if not is_proper_direction():
                print("changing proper direction")
                if is_moving():
                    set_motor_speed(0)
                set_proper_direction()
                # while not await check_constant_speed():
                # time.sleep(loop_Delay / 1000)
                set_motor_speed(1)

            # If distance is fine and remote button isn't pressed and not moving, then start moving
            if current_distance_ahead > sonar_min_distance and not is_moving() \
                    and (moving_right or moving_left or moving_forward or moving_backward) and not stop_everything:
                print("started moving")
                set_motor_speed(1)

            # if not supposed to be moving, but is moving then stop moving
            if ((
                        not moving_backward and not moving_forward and not moving_left and not moving_right) or stop_everything) \
                    and is_moving():
                print("stopping motion")
                set_motor_speed(0)
        else:
            if not was_automated:
                reset_gps_target()
                set_motor_speed(0)
                set_motor_direction(True, True)
                set_motor_direction(False, True)
                was_automated = True
                gps_target = get_next_gps_target()
                finished = False

            current_distance_away = Math.distance_between_points(current_latitude, gps_target[0],
                                                                 current_longitude, gps_target[1])
            direction_target = Math.heading_between_points(current_latitude, gps_target[0],
                                                           current_longitude, gps_target[1])

            if not correct_automated_direction() and not stop_everything and not finished:
                if should_turn_left():
                    set_motor_speed(1 - less_turn_percent, False, True)
                    set_motor_speed(1, False, True)
                else:
                    set_motor_speed(1, False, True)
                    set_motor_speed(1 - less_turn_percent, False, True)
            elif current_distance_ahead >= sonar_min_distance and not stop_everything \
                    and (current_pwm[0] < max_left_pwm or current_pwm[1] < max_right_pwm) \
                    and current_distance_away <= gps_tolerance and not finished:
                set_motor_speed(1)

            if current_distance_away < gps_tolerance and not finished:
                gps_target = get_next_gps_target()
                if gps_target is None:
                    finished = True
                    set_motor_speed(0)

        time.sleep(1 / main_loop_frequency)


try:
    setup()
    print("Setup complete!")
    thread = Background_Thread(web_socket_loop)
    thread3 = Background_Thread(sonar_loop)
    # thread4 = Background_Thread(imu_loop)
    thread2 = Background_Thread(gps_loop)
    main_loop()
except Exception as error:
    set_pwm_freq(False, 0)
    set_pwm_freq(True, 0)
    print("ERROR: " + str(error))
    all_stop = True
    GPIO.cleanup()
    print("cleaned up!")
