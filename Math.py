import math


def two_bytes_to_number(byte_high, byte_low):
    number_result = 256 * byte_high + byte_low
    if number_result >= 32768:
        number_result -= 65536
    return number_result


def distance_between_points(lat1, lat2, lon1, lon2):
    earth_radius = 6371e3  # in meters
    theta_1 = math.radians(lat1)
    theta_2 = math.radians(lat2)
    change_theta = math.radians(lat2 - lat1)
    change_lambda = math.radians(lon2 - lon1)
    square_half_chord_length = (math.sin(change_theta / 2) ** 2) \
                               + ((math.cos(theta_1) * math.cos(theta_2))
                                  * (math.sin(change_lambda / 2) ** 2))
    angular_distance = 2 * math.atan2(math.sqrt(square_half_chord_length), math.sqrt(1 - square_half_chord_length))
    distance = earth_radius * angular_distance
    return distance


def heading_between_points(lat1, lat2, lon1, lon2):
    y = math.sin(lon2 - lon1) * math.cos(lat2)
    x = (math.cos(lat1) * math.sin(lat2)) - (math.sin(lat1) * math.cos(lat2) * math.cos(lon2 - lon1))
    bearing = math.degrees(math.atan2(y, x))
    normalized_bearing = (bearing + 360) % 360
    return normalized_bearing
