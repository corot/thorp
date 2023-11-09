import numpy as np
import rospy
import threading

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from .geometry import distance_2d, yaw_diff


class Tachometer(object):
    """ Record travel statistics:
        - Integrate robot pose to provide mileage (distance and turned angle)
        - Record velocity commands to provide average and max speeds
        - Use actual velocity from odometry to register time stopped and time spinning
    """

    def __init__(self, get_robot_pose_fn, global_frame):
        self._lock = threading.Lock()
        self._running = False
        self._thread = None
        self._prev_pose = None
        self._get_pose = get_robot_pose_fn
        self._global_frame = global_frame
        self._cmd_vel_sub = rospy.Subscriber("cmd_vel", Twist, self._cmd_vel_cb)
        self._cmd_vel_buff = np.empty((0, 2), float)
        self._odometry_sub = rospy.Subscriber("odom", Odometry, self._odometry_cb)
        self._odometry_msg = None
        self._stop_start_t = None
        self._spin_start_t = None
        self._time_stopped = 0.0
        self._time_spinning = 0.0
        self.distance = 0.0
        self.turning = 0.0

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self.update)
        self._thread.start()

    def stop(self):
        if self._running:
            self._running = False
            self._thread.join()
            if self._stop_start_t is not None:
                # If robot is not moving, increment time stopped with the time since last stop started
                self._time_stopped += rospy.get_time() - self._stop_start_t
                self._stop_start_t = None
            if self._spin_start_t is not None:
                # If robot is spinning, increment time spinning with the time since last spin started
                self._time_spinning += rospy.get_time() - self._spin_start_t
                self._spin_start_t = None

    def reset(self):
        self._lock.acquire()
        self._cmd_vel_buff = np.empty((0, 2), float)
        self._stop_start_t = None
        self._spin_start_t = None
        self._time_stopped = 0.0
        self._time_spinning = 0.0
        self._prev_pose = None
        self.distance = 0.0
        self.turning = 0.0
        self._lock.release()

    def update(self):
        while self._running and not rospy.is_shutdown():
            pose = self._get_pose(self._global_frame)
            self._lock.acquire()
            if self._prev_pose:
                self.distance += distance_2d(self._prev_pose, pose)
                self.turning += abs(yaw_diff(self._prev_pose, pose))
            self._prev_pose = pose
            self._lock.release()
            rospy.sleep(0.001)

    @property
    def max_speed(self):
        return np.max(self._cmd_vel_buff[:, 0])

    @property
    def avg_speed(self):
        v = self._cmd_vel_buff[:, 0]
        return np.average(v[v != 0])  # average of all non-zero velocity (that is, we ignore spinning commands)

    @property
    def odometry(self):
        return self._odometry_msg

    @property
    def time_stopped(self):
        return self._time_stopped

    @property
    def time_spinning(self):
        return self._time_spinning

    def _cmd_vel_cb(self, msg):
        if self._running:
            self._cmd_vel_buff = np.append(self._cmd_vel_buff, [[msg.linear.x, msg.angular.z]], axis=0)

    def _odometry_cb(self, msg):
        self._odometry_msg = msg
        if self._running:
            # we are moving if either linear or angular velocities are non-neglectable
            moving = abs(msg.twist.twist.linear.x) > 1e-3 or abs(msg.twist.twist.angular.z) > 1e-3
            # we are spinning if the turning radius ( = v / w) is neglectable
            spinning = abs(msg.twist.twist.linear.x) < abs(msg.twist.twist.angular.z) * 1e-3

            if self._stop_start_t is not None and moving:
                # We start moving: increment time stopped with the time since last stop started
                self._time_stopped += rospy.get_time() - self._stop_start_t
                self._stop_start_t = None
            if self._stop_start_t is None and not moving:
                # We stop moving: register when the stop starts
                self._stop_start_t = rospy.get_time()

            if self._spin_start_t is not None and not spinning:
                # We stop spinning: increment time spinning with the time since last spin started
                self._time_spinning += rospy.get_time() - self._spin_start_t
                self._spin_start_t = None
            if self._spin_start_t is None and spinning:
                # We start spinning: register when the spin starts
                self._spin_start_t = rospy.get_time()
