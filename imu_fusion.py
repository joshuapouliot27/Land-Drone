import time, math
from LIS3MDL import LIS3MDL
from LSM6DS33 import LSM6DS33


class imu_fusion():

    def __init__(self, acc_gyr, magn):

        self.GYRO_GAIN = 0.00875  # Same gain on all axes

        self.Kp_ROLLPITCH = 0.02
        self.Ki_ROLLPITCH = 0.00002
        self.Kp_YAW = 1.2
        self.Ki_YAW = 0.00002

        # CALIBRATION VALUEs
        self.ACCEL_X_MIN = -32764
        self.ACCEL_X_MAX = 32749
        self.ACCEL_Y_MIN = -32764
        self.ACCEL_Y_MAX = 32748
        self.ACCEL_Z_MIN = -32749
        self.ACCEL_Z_MAX = 32764

        self.MAGN_X_MIN = -3091
        self.MAGN_X_MAX = 4233
        self.MAGN_Y_MIN = -5348
        self.MAGN_Y_MAX = 1944
        self.MAGN_Z_MIN = 2001
        self.MAGN_Z_MAX = 9243

        self.GYRO_AVERAGE_OFFSET_X = 446.34455128205127
        self.GYRO_AVERAGE_OFFSET_Y = -574.0839954597049
        self.GYRO_AVERAGE_OFFSET_Z = -723.813596491229

        self.GRAVITY = 256.0  # "1G reference" used for DCM filter and accelerometer calibration

        self.ACCEL_X_OFFSET = ((self.ACCEL_X_MIN + self.ACCEL_X_MAX) / 2.0)
        self.ACCEL_Y_OFFSET = ((self.ACCEL_Y_MIN + self.ACCEL_Y_MAX) / 2.0)
        self.ACCEL_Z_OFFSET = ((self.ACCEL_Z_MIN + self.ACCEL_Z_MAX) / 2.0)
        self.ACCEL_X_SCALE = (self.GRAVITY / (self.ACCEL_X_MAX - self.ACCEL_X_OFFSET))
        self.ACCEL_Y_SCALE = (self.GRAVITY / (self.ACCEL_Y_MAX - self.ACCEL_Y_OFFSET))
        self.ACCEL_Z_SCALE = (self.GRAVITY / (self.ACCEL_Z_MAX - self.ACCEL_Z_OFFSET))

        self.MAGN_X_OFFSET = ((self.MAGN_X_MIN + self.MAGN_X_MAX) / 2.0)
        self.MAGN_Y_OFFSET = ((self.MAGN_Y_MIN + self.MAGN_Y_MAX) / 2.0)
        self.MAGN_Z_OFFSET = ((self.MAGN_Z_MIN + self.MAGN_Z_MAX) / 2.0)
        self.MAGN_X_SCALE = (100.0 / (self.MAGN_X_MAX - self.MAGN_X_OFFSET))
        self.MAGN_Y_SCALE = (100.0 / (self.MAGN_Y_MAX - self.MAGN_Y_OFFSET))
        self.MAGN_Z_SCALE = (100.0 / (self.MAGN_Z_MAX - self.MAGN_Z_OFFSET))

        self.magnetometer = magn
        self.gyroscope_accelerometer = acc_gyr

        # Sensor variables
        self.accelerometer_data = [0.0, 0.0,
                                   0.0]  # Actually stores the NEGATED acceleration (equals gravity, if board not moving).
        self.accelerometer_min = [0.0, 0.0, 0.0]
        self.accelerometer_max = [0.0, 0.0, 0.0]

        self.magnetometer_data = [0.0, 0.0, 0.0]
        self.magnetometer_min = [0.0, 0.0, 0.0]
        self.magnetometer_max = [0.0, 0.0, 0.0]
        self.magnetometer_tmp = [0.0, 0.0, 0.0]

        self.gyro = [0.0, 0.0, 0.0]
        self.gyro_average = [0.0, 0.0, 0.0]
        self.gyro_num_samples = 0

        # DCM variables
        self.mag_heading = 0.0
        self.accelerometer_vector = [0.0, 0.0, 0.0]  # Store the acceleration in a vector
        self.gyroscope_vector = [0.0, 0.0, 0.0]  # Store the gyros turn rate in a vector
        self.omega_vector = [0.0, 0.0, 0.0]  # Corrected Gyro_Vector data

        self.omega_p = [0.0, 0.0, 0.0]  # Omega Proportional correction
        self.omega_i = [0.0, 0.0, 0.0]  # Omega Integrator
        self.omega = [0.0, 0.0, 0.0]
        self.error_roll_pitch = [0.0, 0.0, 0.0]
        self.error_yaw = [0.0, 0.0, 0.0]
        self.dcm_matrix = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        self.update_matrix = [[0.0, 1.0, 2.0], [3.0, 4.0, 5.0], [6.0, 7.0, 8.0]]
        self.temporary_matrix = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]

        # Euler angles
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0

        # DCM timing in the main loop
        self.timestamp = 0.0
        self.timestamp_old = 0.0
        self.G_Dt = 0.0

        # More output-state variables
        self.curr_calibration_sensor = 0.0
        self.reset_calibration_session_flag = True
        self.num_accel_errors = 0.0
        self.num_magn_errors = 0.0
        self.num_gyro_errors = 0.0

        self.setup()

    def to_radians(self, x):
        return x * 0.01745329252

    def gyroscope_scaled_radians(self, x):
        return x * self.to_radians(self.GYRO_GAIN)

    def normalize(self):
        error = 0
        temporary = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
        renorm = 0

        error = -self.vector_dot_product(self.dcm_matrix[0], self.dcm_matrix[1]) * .5  # eq.19

        temporary[0] = self.vector_scale(self.dcm_matrix[1], error)  # eq.19
        temporary[1] = self.vector_scale(self.dcm_matrix[0], error)  # eq.19

        temporary[0] = self.vector_add(temporary[0], self.dcm_matrix[0])  # eq.19
        temporary[1] = self.vector_add(temporary[1], self.dcm_matrix[1])  # eq.19

        temporary[2] = self.vector_cross_product(temporary[0], temporary[1])  # c= a x b #eq.20

        renorm = 0.5 * (3 - self.vector_dot_product(temporary[0], temporary[0]))  # eq.21
        self.dcm_matrix[0] = self.vector_scale(temporary[0], renorm)

        renorm = 0.5 * (3 - self.vector_dot_product(temporary[1], temporary[1]))  # eq.21
        self.dcm_matrix[1] = self.vector_scale(temporary[1], renorm)

        renorm = 0.5 * (3 - self.vector_dot_product(temporary[2], temporary[2]))  # eq.21
        self.dcm_matrix[2] = self.vector_scale(temporary[2], renorm)

    def drift_correction(self):
        # Calculate the magnitude of the accelerometer vector
        accelerometer_magnitude = math.sqrt(
            self.accelerometer_vector[0] * self.accelerometer_vector[0] + self.accelerometer_vector[1] *
            self.accelerometer_vector[1]
            + self.accelerometer_vector[2] * self.accelerometer_vector[2])
        accelerometer_magnitude = accelerometer_magnitude / self.GRAVITY  # Scale to gravity.
        # Dynamic weighting of accelerometer info (reliability filter)
        # Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
        accelerometer_weight = min(max(1 - 2 * abs(1 - accelerometer_magnitude), 1), 0)  #

        error_roll_pitch = self.vector_cross_product(self.accelerometer_vector,
                                                     self.dcm_matrix[2])  # adjust the ground of reference
        self.omega_p = self.vector_scale(error_roll_pitch, self.Kp_ROLLPITCH * accelerometer_weight)

        scaled_omega_i = self.vector_scale(error_roll_pitch, self.Ki_ROLLPITCH * accelerometer_weight)
        self.omega_i = self.vector_add(self.omega_i, scaled_omega_i)

        # *****YAW***************
        # We make the gyro YAW drift correction based on compass magnetic heading

        mag_heading_x = math.cos(self.mag_heading)
        mag_heading_y = math.sin(self.mag_heading)
        error_course = (self.dcm_matrix[0][0] * mag_heading_y) - (
                self.dcm_matrix[1][0] * mag_heading_x)  # Calculating YAW error
        error_yaw = self.vector_scale(self.dcm_matrix[2],
                                      error_course)  # Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.

        scaled_omega_p = self.vector_scale(error_yaw, self.Kp_YAW)  # .01proportional of YAW.
        self.omega_p = self.vector_add(self.omega_p, scaled_omega_p)  # Adding  Proportional.

        scaled_omega_i = self.vector_scale(error_yaw, self.Ki_YAW)  # .00001Integrator
        self.omega_i = self.vector_add(self.omega_i, scaled_omega_i)  # adding integrator to the omega_i

    def matrix_update(self):
        self.gyroscope_vector[0] = self.gyroscope_scaled_radians(self.gyro[0])  # gyro x roll
        self.gyroscope_vector[1] = self.gyroscope_scaled_radians(self.gyro[1])  # gyro y pitch
        self.gyroscope_vector[2] = self.gyroscope_scaled_radians(self.gyro[2])  # gyro z yaw

        self.accelerometer_vector[0] = self.accelerometer_data[0]
        self.accelerometer_vector[1] = self.accelerometer_data[1]
        self.accelerometer_vector[2] = self.accelerometer_data[2]

        self.omega = self.vector_add(self.gyroscope_vector, self.omega_i)  # adding proportional term
        self.omega_vector = self.vector_add(self.omega, self.omega_p)  # adding Integrator term

        # Use drift correction
        self.update_matrix[0][0] = 0
        self.update_matrix[0][1] = -self.G_Dt * self.omega_vector[2]  # -z
        self.update_matrix[0][2] = self.G_Dt * self.omega_vector[1]  # y
        self.update_matrix[1][0] = self.G_Dt * self.omega_vector[2]  # z
        self.update_matrix[1][1] = 0
        self.update_matrix[1][2] = -self.G_Dt * self.omega_vector[0]  # -x
        self.update_matrix[2][0] = -self.G_Dt * self.omega_vector[1]  # -y
        self.update_matrix[2][1] = self.G_Dt * self.omega_vector[0]  # x
        self.update_matrix[2][2] = 0

        self.matrix_multiply(self.dcm_matrix, self.update_matrix, self.temporary_matrix)  # a*b=c

        for x in range(0, 3):
            for y in range(0, 3):
                self.dcm_matrix[x][y] += self.temporary_matrix[x][y]

    def euler_angles(self):
        self.pitch = -math.asin(self.dcm_matrix[2][0])
        self.roll = math.atan2(self.dcm_matrix[2][1], self.dcm_matrix[2][2])
        self.yaw = math.atan2(self.dcm_matrix[1][0], self.dcm_matrix[0][0])

    def compass_heading(self):
        cos_roll = math.cos(self.roll)
        sin_roll = math.sin(self.roll)
        cos_pitch = math.cos(self.pitch)
        sin_pitch = math.sin(self.pitch)

        # Tilt compensated magnetic field X
        mag_x = self.magnetometer_data[0] * cos_pitch + self.magnetometer_data[1] * sin_roll * sin_pitch \
                + self.magnetometer_data[2] * cos_roll * sin_pitch
        # Tilt compensated magnetic field Y
        mag_y = self.magnetometer_data[1] * cos_roll - self.magnetometer_data[2] * sin_roll
        # Magnetic Heading
        self.mag_heading = math.atan2(-mag_y, mag_x)

    # Computes the dot product of two vectors
    @staticmethod
    def vector_dot_product(v1, v2):
        result = 0.0
        for c in range(0, 3):
            result += v1[c] * v2[c]
        return result

    # Computes the cross product of two vectors
    # out has to different from v1 and v2 (no in-place)!
    @staticmethod
    def vector_cross_product(v1, v2):
        out = [0, 0, 0]
        out[0] = (v1[1] * v2[2]) - (v1[2] * v2[1])
        out[1] = (v1[2] * v2[0]) - (v1[0] * v2[2])
        out[2] = (v1[0] * v2[1]) - (v1[1] * v2[0])
        return out

    # Multiply the vector by a scalar
    @staticmethod
    def vector_scale(v, scale):
        out = [0, 0, 0]
        for c in range(0, 3):
            out[c] = v[c] * scale
        return out

    # Adds two vectors
    @staticmethod
    def vector_add(v1, v2):
        out = [0, 0, 0]
        for c in range(0, 3):
            out[c] = v1[c] + v2[c]
        return out

    # Multiply two 3x3 matrices: out = a * b
    # out has to different from a and b (no in-place)!
    @staticmethod
    def matrix_multiply(a, b, out):
        for x in range(0, 3):
            for y in range(0, 3):
                out[x][y] = a[x][0] * b[0][y] + a[x][1] * b[1][y] + a[x][2] * b[2][y]

    # Multiply 3x3 matrix with vector: out = a * b
    # out has to different from b (no in-place)!
    @staticmethod
    def matrix_vector_multiply(a, b, out):
        for x in range(0, 3):
            out[x] = a[x][0] * b[0] + a[x][1] * b[1] + a[x][2] * b[2]

    # Init rotation matrix using euler angles
    @staticmethod
    def init_rotation_matrix(m, yaw, pitch, roll):
        c1 = math.cos(roll)
        s1 = math.sin(roll)
        c2 = math.cos(pitch)
        s2 = math.sin(pitch)
        c3 = math.cos(yaw)
        s3 = math.sin(yaw)

        m[0][0] = c2 * c3
        m[0][1] = c3 * s1 * s2 - c1 * s3
        m[0][2] = s1 * s3 + c1 * c3 * s2

        m[1][0] = c2 * s3
        m[1][1] = c1 * c3 + s1 * s2 * s3
        m[1][2] = c1 * s2 * s3 - c3 * s1

        m[2][0] = -s2
        m[2][1] = c2 * s1
        m[2][2] = c1 * c2

    @staticmethod
    def to_degrees(x):
        return x * 57.2957795131  # *180/pi

    def output_angles(self):
        print("#YPR= " + str(int(self.to_degrees(self.yaw))) + " " + str(int(self.to_degrees(self.pitch)))
              + " " + str(int(self.to_degrees(self.roll))))

    def read_sensors(self):
        gyro_data = self.gyroscope_accelerometer.get_gyroscope_data()
        self.gyro = [gyro_data.x, gyro_data.y, gyro_data.z]
        accel_data = self.gyroscope_accelerometer.get_accelerometer_data()
        self.accelerometer_data = [accel_data.x, accel_data.y, accel_data.z]  # Read accelerometer
        magn_data = self.magnetometer.get_magnetometer_data()
        self.magnetometer_data = [magn_data.x, magn_data.y, magn_data.z]  # Read magnetometer

    # Read every sensor and record a time stamp
    # Init DCM with unfiltered orientation
    # TODO re-init global vars?

    @staticmethod
    def millis():
        return round(time.time() * 1000)

    def reset_sensor_fusion(self):
        x_axis = [1.0, 0.0, 0.0]

        self.read_sensors()
        self.timestamp = self.millis()

        # GET PITCH
        # Using y-z-plane-component/x-component of gravity vector
        self.pitch = -math.atan2(self.accelerometer_data[0],
                                 math.sqrt(
                                     self.accelerometer_data[1] * self.accelerometer_data[1] + self.accelerometer_data[
                                         2] * self.accelerometer_data[2]))

        # GET ROLL
        # Compensate pitch of gravity vector
        temp1 = self.vector_cross_product(self.accelerometer_data, x_axis)
        temp2 = self.vector_cross_product(x_axis, temp1)
        # Normally using x-z-plane-component/y-component of compensated gravity vector
        # roll = atan2(temp2[1], sqrt(temp2[0] * temp2[0] + temp2[2] * temp2[2]));
        # Since we compensated for pitch, x-z-plane-component equals z-component:
        self.roll = math.atan2(temp2[1], temp2[2])

        # GET YAW
        self.compass_heading()
        self.yaw = self.mag_heading

        # Init rotation matrix
        self.init_rotation_matrix(self.dcm_matrix, self.yaw, self.pitch, self.roll)

    # Apply calibration to raw sensor readings
    def compensate_sensor_errors(self):
        self.accelerometer_data[0] = (self.accelerometer_data[0] - self.ACCEL_X_OFFSET) * self.ACCEL_X_SCALE
        self.accelerometer_data[1] = (self.accelerometer_data[1] - self.ACCEL_Y_OFFSET) * self.ACCEL_Y_SCALE
        self.accelerometer_data[2] = (self.accelerometer_data[2] - self.ACCEL_Z_OFFSET) * self.ACCEL_Z_SCALE

        self.magnetometer_data[0] = (self.magnetometer_data[0] - self.MAGN_X_OFFSET) * self.MAGN_X_SCALE
        self.magnetometer_data[1] = (self.magnetometer_data[1] - self.MAGN_Y_OFFSET) * self.MAGN_Y_SCALE
        self.magnetometer_data[2] = (self.magnetometer_data[2] - self.MAGN_Z_OFFSET) * self.MAGN_Z_SCALE

        # Compensate gyroscope error
        self.gyro[0] -= self.GYRO_AVERAGE_OFFSET_X
        self.gyro[1] -= self.GYRO_AVERAGE_OFFSET_Y
        self.gyro[2] -= self.GYRO_AVERAGE_OFFSET_Z

    # Reset calibration session if reset_calibration_session_flag is set
    def check_reset_calibration_session(self):
        # Raw sensor values have to be read already, but no error compensation applied
        # Reset this calibration session?
        if not self.reset_calibration_session_flag:
            return

        # Reset acc and mag calibration variables
        for i in range(0, 3):
            self.accelerometer_min[i] = self.accelerometer_data[i]
            self.accelerometer_max[i] = self.accelerometer_data[i]
            self.magnetometer_min[i] = self.magnetometer_data[i]
            self.magnetometer_max[i] = self.magnetometer_data[i]

        # Reset gyro calibration variables
        self.gyro_num_samples = 0  # Reset gyro calibration averaging
        self.gyro_average[0] = 0.0
        self.gyro_average[1] = 0.0
        self.gyro_average[2] = 0.0

        self.reset_calibration_session_flag = False

    def get_true_heading(self):
        magnetometer_x_component = self.magnetometer_data[0] * math.cos(self.pitch) \
                                   + self.magnetometer_data[2] * math.sin(self.pitch)
        magnetometer_y_component = self.magnetometer_data[0] * math.sin(self.roll) * math.sin(self.pitch) \
                                   + self.magnetometer_data[1] * math.cos(self.roll) \
                                   - self.magnetometer_data[2] * math.sin(self.roll) * math.cos(self.pitch)
        tilt_compensated_heading = math.degrees(math.atan2(magnetometer_y_component, magnetometer_x_component))
        if tilt_compensated_heading < 0:
            tilt_compensated_heading += 360
        return tilt_compensated_heading

    def setup(self):
        self.read_sensors()
        # Read sensors, init DCM algorithm
        time.sleep(2)  # Give sensors enough time to collect data
        self.reset_sensor_fusion()

    # Main loop
    def loop(self):
        # Time to read the sensors again?
        if (self.millis() - self.timestamp) >= 1000:
            self.timestamp_old = self.timestamp
            self.timestamp = self.millis()
            if self.timestamp > self.timestamp_old:
                self.G_Dt = (self.timestamp - self.timestamp_old) / 1000.0
            else:
                self.G_Dt = 0

            # Update sensor readings
            self.read_sensors()
            # Apply sensor calibration
            self.compensate_sensor_errors()

            # Run DCM algorithm
            self.compass_heading()  # Calculate magnetic heading
            self.matrix_update()
            self.normalize()
            self.drift_correction()
            self.euler_angles()

            self.output_angles()
