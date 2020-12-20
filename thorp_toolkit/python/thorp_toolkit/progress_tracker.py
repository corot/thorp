from visualization import Visualization
from geometry import distance_2d


class ProgressTracker:
    """ Tracks robot progression along a sequence of waypoints """

    def __init__(self, waypoints, reached_threshold):
        self._waypoints = waypoints
        self._reached_threshold = reached_threshold
        self._next_wp = 0
        self._reached = False
        self._min_dist = float('inf')

    def reset(self):
        self._next_wp = 0
        self._reached = False
        self._min_dist = float('inf')
        Visualization().delete_markers()

    def update_pose(self, robot_pose):
        """
        Update progress with a new robot pose.
        :param robot_pose: Current robot pose (stamped)
        """
        if self._next_wp is None:
            return  # already arrived
        dist = distance_2d(robot_pose, self._waypoints[self._next_wp])
        if self._reached and dist > self._min_dist:
            Visualization().add_disc_marker(self._waypoints[self._next_wp], (0.2, 0.2, 0.1))
            Visualization().publish_markers()
            self._next_wp += 1
            if self._next_wp < len(self._waypoints):
                # go for next waypoint
                self._reached = False
                self._min_dist = float('inf')
            else:
                # arrived; clear reached waypoints markers
                Visualization().delete_markers()
                self._next_wp = None
            return

        self._min_dist = min(self._min_dist, dist)
        if dist <= self._reached_threshold:
            self._reached = True

    def next_waypoint(self):
        return self._next_wp

    def reached_waypoint(self):
        if self._next_wp is None:
            return len(self._waypoints) - 1
        elif self._next_wp == 0:
            return None
        return self._next_wp - 1
