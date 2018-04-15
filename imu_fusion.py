import time, math
from LIS3MDL import LIS3MDL
from LSM6DS33 import LSM6DS33

class imu_fusion():

    def __init__(self, acc_gyr, magn):

        self.GYRO_GAIN = 0.06957  # Same gain on all axes

        self.Kp_ROLLPITCH = 0.02
        self.Ki_ROLLPITCH = 0.00002
        self.Kp_YAW = 1.2
        self.Ki_YAW = 0.00002

        # CALIBRATION VALUEs
        self.ACCEL_X_MIN = -289
        self.ACCEL_X_MAX = 294
        self.ACCEL_Y_MIN = -268
        self.ACCEL_Y_MAX = 288
        self.ACCEL_Z_MIN = -294
        self.ACCEL_Z_MAX = 269

        self.MAGN_X_MIN = -600
        self.MAGN_X_MAX = 600
        self.MAGN_Y_MIN = -600
        self.MAGN_Y_MAX = 600
        self.MAGN_Z_MIN = -600
        self.MAGN_Z_MAX = 600

        self.GYRO_AVERAGE_OFFSET_X = 0
        self.GYRO_AVERAGE_OFFSET_Y = 0
        self.GYRO_AVERAGE_OFFSET_Z = 0

        self.CALIBRATION__MAGN_USE_EXTENDED = False
        self.magn_ellipsoid_center = [3.80526, -16.4455, 87.4052]
        self.magn_ellipsoid_transform = [[0.970991, 0.00583310, -0.00265756], [0.00583310, 0.952958, 2.76726e-05],
                                    [-0.00265756, 2.76726e-05, 0.999751]]

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

        self.magn = magn
        self.gyro_accel = acc_gyr

        # Sensor variables
        self.accel = [0, 0, 0]  # Actually stores the NEGATED acceleration (equals gravity, if board not moving).
        self.accel_min = [0, 0, 0]
        self.accel_max = [0, 0, 0]

        self.magnetom = [0, 0, 0]
        self.magnetom_min = [0, 0, 0]
        self.magnetom_max = [0, 0, 0]
        self.magnetom_tmp = [0, 0, 0]

        self.gyro = [0, 0, 0]
        self.gyro_average = [0, 0, 0]
        self.gyro_num_samples = 0

        # DCM variables
        self.MAG_Heading = 0.0
        self.Accel_Vector = [0, 0, 0]  # Store the acceleration in a vector
        self.Gyro_Vector = [0, 0, 0]  # Store the gyros turn rate in a vector
        self.Omega_Vector = [0, 0, 0]  # Corrected Gyro_Vector data

        self.Omega_P = [0, 0, 0]  # Omega Proportional correction
        self.Omega_I = [0, 0, 0]  # Omega Integrator
        self.Omega = [0, 0, 0]
        self.errorRollPitch = [0, 0, 0]
        self.errorYaw = [0, 0, 0]
        self.DCM_Matrix = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
        self.Update_Matrix = [[0, 1, 2], [3, 4, 5], [6, 7, 8]]
        self.Temporary_Matrix = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]

        # Euler angles
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0

        # DCM timing in the main loop\
        self.timestamp = 0
        self.timestamp_old = 0
        self.G_Dt = 0.0

        # More output-state variables
        self.curr_calibration_sensor = 0
        self.reset_calibration_session_flag = True
        self.num_accel_errors = 0
        self.num_magn_errors = 0
        self.num_gyro_errors = 0

        self.setup()


    def TO_RAD(self, x):
        return x * 0.01745329252


    def GYRO_SCALED_RAD(self, x):
        return x * self.TO_RAD(self.GYRO_GAIN)


    def Normalize(self):
        error = 0
        temporary = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
        renorm = 0

        error = -self.Vector_Dot_Product(self.DCM_Matrix[0], self.DCM_Matrix[1]) * .5  # eq.19

        temporary[0] = self.Vector_Scale(self.DCM_Matrix[1], error)  # eq.19
        temporary[1] = self.Vector_Scale(self.DCM_Matrix[0], error)  # eq.19

        temporary[0] = self.Vector_Add(temporary[0], self.DCM_Matrix[0])  # eq.19
        temporary[1] = self.Vector_Add(temporary[1], self.DCM_Matrix[1])  # eq.19

        temporary[2] = self.Vector_Cross_Product(temporary[0], temporary[1])  # c= a x b #eq.20

        renorm = 0.5 * (3 - self.Vector_Dot_Product(temporary[0], temporary[0]))  # eq.21
        self.DCM_Matrix[0] = self.Vector_Scale(temporary[0], renorm)

        renorm = 0.5 * (3 - self.Vector_Dot_Product(temporary[1], temporary[1]))  # eq.21
        self.DCM_Matrix[1] = self.Vector_Scale(temporary[1], renorm)

        renorm = 0.5 * (3 - self.Vector_Dot_Product(temporary[2], temporary[2]))  # eq.21
        self.DCM_Matrix[2] = self.Vector_Scale(temporary[2], renorm)


    def Drift_correction(self):
        mag_heading_x = 0
        mag_heading_y = 0
        # Compensation the Roll, Pitch and Yaw drift.
        Scaled_Omega_P = [0, 0, 0]
        Scaled_Omega_I = [0, 0, 0]
        Accel_magnitude = 0
        Accel_weight = 0

        # *****Roll and Pitch***************

        # Calculate the magnitude of the accelerometer vector
        Accel_magnitude = math.sqrt(
            self.Accel_Vector[0] * self.Accel_Vector[0] + self.Accel_Vector[1] * self.Accel_Vector[1]
            + self.Accel_Vector[2] * self.Accel_Vector[2]);
        Accel_magnitude = Accel_magnitude / self.GRAVITY;  # Scale to gravity.
        # Dynamic weighting of accelerometer info (reliability filter)
        # Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
        Accel_weight = min(max(1 - 2 * abs(1 - Accel_magnitude), 1), 0)  #

        errorRollPitch = self.Vector_Cross_Product(self.Accel_Vector, self.DCM_Matrix[2])  # adjust the ground of reference
        Omega_P = self.Vector_Scale(errorRollPitch, self.Kp_ROLLPITCH * Accel_weight)

        Scaled_Omega_I = self.Vector_Scale(errorRollPitch, self.Ki_ROLLPITCH * Accel_weight)
        Omega_I = self.Vector_Add(self.Omega_I, Scaled_Omega_I)

        # *****YAW***************
        # We make the gyro YAW drift correction based on compass magnetic heading

        mag_heading_x = math.cos(self.MAG_Heading)
        mag_heading_y = math.sin(self.MAG_Heading)
        errorCourse = (self.DCM_Matrix[0][0] * mag_heading_y) - (self.DCM_Matrix[1][0] * mag_heading_x)  # Calculating YAW error
        errorYaw = self.Vector_Scale(self.DCM_Matrix[2],
                            errorCourse)  # Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.

        Scaled_Omega_P = self.Vector_Scale(errorYaw, self.Kp_YAW)  # .01proportional of YAW.
        self.Omega_P = self.Vector_Add(Omega_P, Scaled_Omega_P)  # Adding  Proportional.

        Scaled_Omega_I = self.Vector_Scale(errorYaw, self.Ki_YAW)  # .00001Integrator
        self.Omega_I = self.Vector_Add(Omega_I, Scaled_Omega_I)  # adding integrator to the Omega_I


    def Matrix_update(self):
        self.Gyro_Vector[0] = self.GYRO_SCALED_RAD(self.gyro[0])  # gyro x roll
        self.Gyro_Vector[1] = self.GYRO_SCALED_RAD(self.gyro[1])  # gyro y pitch
        self.Gyro_Vector[2] = self.GYRO_SCALED_RAD(self.gyro[2])  # gyro z yaw

        self.Accel_Vector[0] = self.accel[0]
        self.Accel_Vector[1] = self.accel[1]
        self. Accel_Vector[2] = self.accel[2]

        self.Omega = self.Vector_Add(self.Gyro_Vector, self.Omega_I)  # adding proportional term
        self.Omega_Vector = self.Vector_Add(self.Omega, self.Omega_P)  # adding Integrator term

        # Use drift correction
        self.Update_Matrix[0][0] = 0
        self.Update_Matrix[0][1] = -self.G_Dt * self.Omega_Vector[2]  # -z
        self.Update_Matrix[0][2] = self.G_Dt * self.Omega_Vector[1]  # y
        self.Update_Matrix[1][0] = self.G_Dt * self.Omega_Vector[2]  # z
        self.Update_Matrix[1][1] = 0
        self.Update_Matrix[1][2] = -self.G_Dt * self.Omega_Vector[0]  # -x
        self.Update_Matrix[2][0] = -self.G_Dt * self.Omega_Vector[1]  # -y
        self.Update_Matrix[2][1] = self.G_Dt * self.Omega_Vector[0]  # x
        self.Update_Matrix[2][2] = 0

        self.Matrix_Multiply(self.DCM_Matrix, self.Update_Matrix, self.Temporary_Matrix)  # a*b=c

        for x in range(0, 3):
            for y in range(0, 3):
                self.DCM_Matrix[x][y] += self.Temporary_Matrix[x][y]


    def Euler_angles(self):
        self.pitch = -math.asin(self.DCM_Matrix[2][0])
        self.roll = math.atan2(self.DCM_Matrix[2][1], self.DCM_Matrix[2][2])
        self.yaw = math.atan2(self.DCM_Matrix[1][0], self.DCM_Matrix[0][0])


    def Compass_Heading(self):
        cos_roll = math.cos(self.roll)
        sin_roll = math.sin(self.roll)
        cos_pitch = math.cos(self.pitch)
        sin_pitch = math.sin(self.pitch)

        # Tilt compensated magnetic field X
        mag_x = self.magnetom[0] * cos_pitch + self.magnetom[1] * sin_roll * sin_pitch + self.magnetom[2] * cos_roll * sin_pitch
        # Tilt compensated magnetic field Y
        mag_y = self.magnetom[1] * cos_roll - self.magnetom[2] * sin_roll
        # Magnetic Heading
        MAG_Heading = math.atan2(-mag_y, mag_x)


    # Computes the dot product of two vectors
    def Vector_Dot_Product(self, v1, v2):
        result = 0.0
        for c in range(0, 3):
            result += v1[c] * v2[c]
        return result


    # Computes the cross product of two vectors
    # out has to different from v1 and v2 (no in-place)!
    def Vector_Cross_Product(self, v1, v2):
        out = [0, 0, 0]
        out[0] = (v1[1] * v2[2]) - (v1[2] * v2[1])
        out[1] = (v1[2] * v2[0]) - (v1[0] * v2[2])
        out[2] = (v1[0] * v2[1]) - (v1[1] * v2[0])
        return out


    # Multiply the vector by a scalar
    def Vector_Scale(self, v, scale):
        out = [0, 0, 0]
        for c in range(0, 3):
            out[c] = v[c] * scale
        return out


    # Adds two vectors
    def Vector_Add(self, v1, v2):
        out = [0, 0, 0]
        for c in range(0, 3):
            out[c] = v1[c] + v2[c]
        return out


    # Multiply two 3x3 matrices: out = a * b
    # out has to different from a and b (no in-place)!
    def Matrix_Multiply(self, a, b, out):
        for x in range(0, 3):
            for y in range(0, 3):
                out[x][y] = a[x][0] * b[0][y] + a[x][1] * b[1][y] + a[x][2] * b[2][y]


    # Multiply 3x3 matrix with vector: out = a * b
    # out has to different from b (no in-place)!
    def Matrix_Vector_Multiply(self, a, b, out):
        for x in range(0, 3):
            out[x] = a[x][0] * b[0] + a[x][1] * b[1] + a[x][2] * b[2]


    # Init rotation matrix using euler angles
    def init_rotation_matrix(self, m, yaw, pitch, roll):
        c1 = math.cos(roll)
        s1 = math.sin(roll)
        c2 = math.cos(pitch)
        s2 = math.sin(pitch);
        c3 = math.cos(yaw)
        s3 = math.sin(yaw)

        m[0][0] = c2 * c3;
        m[0][1] = c3 * s1 * s2 - c1 * s3
        m[0][2] = s1 * s3 + c1 * c3 * s2

        m[1][0] = c2 * s3;
        m[1][1] = c1 * c3 + s1 * s2 * s3
        m[1][2] = c1 * s2 * s3 - c3 * s1

        m[2][0] = -s2
        m[2][1] = c2 * s1
        m[2][2] = c1 * c2


    def TO_DEG(self, x):
        return (x * 57.2957795131)  # *180/pi


    def output_angles(self):
        print("#YPR= " + str(int(self.TO_DEG(self.yaw))) + " " + str(int(self.TO_DEG(self.pitch)))
              + " " + str(int(self.TO_DEG(self.roll))))

    def read_sensors(self):
        gyro_data = self.gyro_accel.get_gyroscope_data()
        self.gyro = (gyro_data.x, gyro_data.y, gyro_data.z)
        accel_data = self.gyro_accel.get_accelerometer_data()
        self.accel = (accel_data.x, accel_data.y, accel_data.z)  # Read accelerometer
        magn_data = self.magn.get_magnetometer_data()
        self.magnetom = (magn_data.x, magn_data.y, magn_data.z)  # Read magnetometer


    # Read every sensor and record a time stamp
    # Init DCM with unfiltered orientation
    # TODO re-init global vars?

    def millis(self):
        return round(time.time() * 1000)


    def reset_sensor_fusion(self):
        temp1 = [0, 0, 0]
        temp2 = [0, 0, 0]
        xAxis = [1.0, 0.0, 0.0]

        self.read_sensors()
        self.timestamp = self.millis()

        # GET PITCH
        # Using y-z-plane-component/x-component of gravity vector
        self.pitch = -math.atan2(self.accel[0], math.sqrt(self.accel[1] * self.accel[1] + self.accel[2] * self.accel[2]))

        # GET ROLL
        # Compensate pitch of gravity vector
        temp1 = self.Vector_Cross_Product(self.accel, xAxis)
        temp2 = self.Vector_Cross_Product(xAxis, temp1)
        # Normally using x-z-plane-component/y-component of compensated gravity vector
        # roll = atan2(temp2[1], sqrt(temp2[0] * temp2[0] + temp2[2] * temp2[2]));
        # Since we compensated for pitch, x-z-plane-component equals z-component:
        self.roll = math.atan2(temp2[1], temp2[2])

        # GET YAW
        self.Compass_Heading()
        self.yaw = self.MAG_Heading

        # Init rotation matrix
        self.init_rotation_matrix(self.DCM_Matrix, self.yaw, self.pitch, self.roll)


    # Apply calibration to raw sensor readings
    def compensate_sensor_errors(self):
        self.accel[0] = (self.accel[0] - self.ACCEL_X_OFFSET) * self.ACCEL_X_SCALE
        self.accel[1] = (self.accel[1] - self.ACCEL_Y_OFFSET) * self.ACCEL_Y_SCALE
        self.accel[2] = (self.accel[2] - self.ACCEL_Z_OFFSET) * self.ACCEL_Z_SCALE

        # Compensate magnetometer error
        if self.CALIBRATION__MAGN_USE_EXTENDED:
            magentom_tmp = self.magnetom.copy()
            for i in range(0, 3):
                magentom_tmp[i] = self.magnetom[i] - self.magn_ellipsoid_center[i]
                self.Matrix_Vector_Multiply(self.magn_ellipsoid_transform, magentom_tmp, self.magnetom)
        else:
            self.magnetom[0] = (self.magnetom[0] - self.MAGN_X_OFFSET) * self.MAGN_X_SCALE
            self.magnetom[1] = (self.magnetom[1] - self.MAGN_Y_OFFSET) * self.MAGN_Y_SCALE
            self.magnetom[2] = (self.magnetom[2] - self.MAGN_Z_OFFSET) * self.MAGN_Z_SCALE

        # Compensate gyroscope error
        self.gyro[0] -= self.GYRO_AVERAGE_OFFSET_X
        self.gyro[1] -= self.GYRO_AVERAGE_OFFSET_Y
        self.gyro[2] -= self.GYRO_AVERAGE_OFFSET_Z


    # Reset calibration session if reset_calibration_session_flag is set
    def check_reset_calibration_session(self):
        # Raw sensor values have to be read already, but no error compensation applied
        # Reset this calibration session?
        if self.reset_calibration_session_flag == False:
            return

        # Reset acc and mag calibration variables
        for i in range(0, 3):
            self.accel_min[i] = self.accel_max[i] = self.accel[i]
            self.magnetom_min[i] = self.magnetom_max[i] = self.magnetom[i]

        # Reset gyro calibration variables
        self.gyro_num_samples = 0  # Reset gyro calibration averaging
        self.gyro_average[0] = self.gyro_average[1] = self.gyro_average[2] = 0.0

        self.reset_calibration_session_flag = False

    def get_heading(self):
        return self


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
            self.Compass_Heading()  # Calculate magnetic heading
            self.Matrix_update()
            self.Normalize()
            self.Drift_correction()
            self.Euler_angles()

            self.output_angles()
