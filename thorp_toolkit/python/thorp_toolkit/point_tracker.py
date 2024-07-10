import numpy as np
import matplotlib.pyplot as plt
from filterpy.kalman import KalmanFilter


class PointTracker:
    def __init__(self, initial_pos=None, dt=0.1):
        if initial_pos is None:
            initial_pos = 0, 0
        self.kf = self.create_kalman_filter(initial_pos, dt)
        self.estimated_positions = [initial_pos]

    def create_kalman_filter(self, initial_pos, dt):
        kf = KalmanFilter(dim_x=5, dim_z=3)

        # State transition matrix
        kf.F = np.array([[1, 0, dt, 0, 0],
                         [0, 1, 0, dt, 0],
                         [0, 0, 1, 0, 0],
                         [0, 0, 0, 1, 0],
                         [0, 0, 0, 0, 1]])

        # Measurement function
        kf.H = np.array([[1, 0, 0, 0, 0],
                         [0, 1, 0, 0, 0],
                         [0, 0, 0, 0, 1]])

        # Initial state
        kf.x = np.array([*initial_pos, 0, 0, 0])  # x, y, vx, vy, theta

        # Covariance matrix
        kf.P *= 1000

        # Process noise
        kf.Q = np.eye(5) * 0.1

        # Measurement noise
        kf.R = np.eye(3) * 5

        return kf

    def update(self, point):
        """
        Update the Kalman filter with a new point and return the estimated state.

        Args:
            point (tuple): A tuple (x, y) representing the new point.

        Returns:
            tuple: The estimated state (x, y, theta) after updating with the new point.
        """
        if len(self.estimated_positions) > 0:
            last_point = self.estimated_positions[-1]
            theta = np.arctan2(point[1] - last_point[1], point[0] - last_point[0])
        else:
            theta = 0

        measurement = np.array([point[0], point[1], theta])

        self.kf.predict()
        self.kf.update(measurement)

        estimated_position = self.kf.x[0], self.kf.x[1], self.kf.x[4]
        self.estimated_positions.append(estimated_position)

        return estimated_position

    def predict(self):
        self.kf.predict()
        return self.kf.x

    def reset(self):
        self.kf.x = np.array([0, 0, 0, 0, 0])

    def predict_future_positions(self, num_predictions=10):
        future_positions = []
        for _ in range(num_predictions):
            self.kf.predict()
            future_positions.append((self.kf.x[0], self.kf.x[1], self.kf.x[4]))
        return future_positions

    def plot_results(self, points, future_positions):
        plt.figure()
        plt.scatter(points[:, 0], points[:, 1], color='blue', label='Measured points')
        plt.plot(*zip(*[(p[0], p[1]) for p in self.estimated_positions]), color='green', label='Estimated trajectory')
        plt.plot(*zip(*[(p[0], p[1]) for p in future_positions]), color='red', linestyle='--',
                 label='Predicted trajectory')
        plt.quiver(*zip(*[(p[0], p[1]) for p in self.estimated_positions]),
                   np.cos([p[2] for p in self.estimated_positions]),
                   np.sin([p[2] for p in self.estimated_positions]),
                   color='green')
        plt.quiver(*zip(*[(p[0], p[1]) for p in future_positions]),
                   np.cos([p[2] for p in future_positions]),
                   np.sin([p[2] for p in future_positions]),
                   color='red', linestyle='--')
        plt.legend()
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Kalman Filter - Future Pose and Direction Estimation')
        plt.show()


# SAMPLE USAGE
#
# Sample points (e.g., from a tracked object)
# points = np.array([[11.11, 1], [11.12, 2], [11.13, 3], [11.14, 4], [11.15, 5]])
#
# tracker = PointTracker()
#
# # Update tracker with each point and estimate future positions after each update
# for point in points:
#     tracker.update(point)
#
# # Predict future positions
# future_positions = tracker.predict_future_positions()
#
# tracker.plot_results(points, future_positions)

#
# points = np.array([[1, 1], [2, 2], [3, 3], [4, 4], [5, 5]])
#
# tracker = PointTracker()
#
# future_positions = []
# # Update tracker with each point and estimate future positions after each update
# for point in points:
#     tracker.update(point)
#
# kk = tracker.predict()
# for point in points:
#     future_positions.append(kk)
#
# # Predict future positions
# #future_positions = tracker.predict_future_positions()
#
# tracker.plot_results(points, future_positions)
