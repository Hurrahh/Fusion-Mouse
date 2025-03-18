import numpy as np

class KalmanFilter:
    def __init__(self, Q_angle=0.001, Q_bias=0.003, R_measure=0.03):
        """
        Initialize Kalman filter parameters.
        :param Q_angle: Process noise variance for the accelerometer angle.
        :param Q_bias: Process noise variance for the gyro bias.
        :param R_measure: Measurement noise variance.
        """
        self.Q_angle = Q_angle
        self.Q_bias = Q_bias
        self.R_measure = R_measure

        self.angle = 0.0
        self.bias = 0.0
        self.P = np.array([[0.0, 0.0], [0.0, 0.0]])

    def update(self, new_angle, new_rate, dt):
        """
        Update the Kalman filter with new measurements.
        :param new_angle: Measured angle from the accelerometer/magnetometer.
        :param new_rate: Measured rate from the gyroscope.
        :param dt: Time step.
        :return: Filtered angle.
        """
        rate = new_rate - self.bias
        self.angle += dt * rate

        # Update error covariance matrix
        self.P[0][0] += dt * (dt * self.P[1][1] - self.P[0][1] - self.P[1][0] + self.Q_angle)
        self.P[0][1] -= dt * self.P[1][1]
        self.P[1][0] -= dt * self.P[1][1]
        self.P[1][1] += self.Q_bias * dt

        # Compute Kalman gain
        S = self.P[0][0] + self.R_measure
        K = [self.P[0][0] / S, self.P[1][0] / S]

        # Update angle and bias with new measurements
        y = new_angle - self.angle
        self.angle += K[0] * y
        self.bias += K[1] * y

        # Update error covariance matrix
        P00_temp = self.P[0][0]
        P01_temp = self.P[0][1]

        self.P[0][0] -= K[0] * P00_temp
        self.P[0][1] -= K[0] * P01_temp
        self.P[1][0] -= K[1] * P00_temp
        self.P[1][1] -= K[1] * P01_temp

        return self.angle

def calculate_orientation(accel, gyro, dt, kalman_filters):
    # Calculate roll and pitch from accelerometer
    accel_x, accel_y, accel_z = accel
    roll_accel = np.arctan2(accel_y, accel_z) * 180 / np.pi
    pitch_accel = np.arctan2(-accel_x, np.sqrt(accel_y**2 + accel_z**2)) * 180 / np.pi

    # Update Kalman filters
    roll = kalman_filters[0].update(roll_accel, gyro[0], dt)
    pitch = kalman_filters[1].update(pitch_accel, gyro[1], dt)

    # Calculate yaw by integrating gyroscope data
    yaw_rate = gyro[2]
    yaw_increment = yaw_rate * dt
    yaw_new = kalman_filters[2].angle + yaw_increment

    yaw = kalman_filters[2].update(yaw_new, gyro[2], dt)  # No external reference for yaw, only gyroscope

    return pitch, roll, yaw

# Example usage
if __name__ == "__main__":
    # Simulated IMU data (replace with actual sensor readings)
    accel_data = [0.0, 0.0, 9.8]  # Accelerometer (x, y, z)
    gyro_data = [0.1, 0.2, 0.3]   # Gyroscope (x, y, z)
    dt = 0.01                     # Time step (10 ms)

    # Initialize Kalman filters for roll, pitch, and yaw
    kalman_roll = KalmanFilter()
    kalman_pitch = KalmanFilter()
    kalman_yaw = KalmanFilter()

    kalman_filters = [kalman_roll, kalman_pitch, kalman_yaw]

    yaw_accumulated = 0.0

    pitch, roll, yaw = calculate_orientation(accel_data, gyro_data, dt, kalman_filters)

    print(f"Pitch: {pitch:.2f}, Roll: {roll:.2f}, Yaw: {yaw:.2f}, Accumulated Yaw: {yaw_accumulated:.2f}")
