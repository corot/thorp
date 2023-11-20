import rospy

from random import random
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import PoseStamped
from jsk_rviz_plugins.msg import OverlayText
from visualization_msgs.msg import Marker, MarkerArray

from .singleton import Singleton


class Visualization(metaclass=Singleton):
    """ Singleton providing assistance to create and publish visual markers """

    def __init__(self, topic='~visual_markers', lifetime=60):
        self._marker_pub = rospy.Publisher(topic, MarkerArray, queue_size=1)
        self._lifetime = lifetime
        self._markers_array = []
        self._active_markers = []  # published markers
        rospy.sleep(0.25)  # wait a moment until the publisher is ready

    def publish_markers(self, start_id=1):
        """ Publish marker_array and write lifetime and id """
        if not self._markers_array:
            return
        duration = rospy.Duration(self._lifetime)
        for i, marker in enumerate(self._markers_array):
            marker.lifetime = duration
            marker.id = start_id + i
        if len(self._active_markers) > len(self._markers_array):
            # need to delete some markers
            offset = len(self._active_markers) - len(self._markers_array)
            for marker in self._active_markers[offset:]:
                marker.action = marker.DELETE
            self._marker_pub.publish(self._markers_array + self._active_markers[offset:])
        else:
            self._marker_pub.publish(self._markers_array)
        self._active_markers = self._markers_array

    def reset(self):
        """ Delete all markers and clear the array """
        self.delete_markers()
        self._markers_array = []

    def delete_markers(self):
        """ Delete all markers """
        for marker in self._active_markers:
            marker.action = marker.DELETE
        self._marker_pub.publish(self._active_markers)
        self._active_markers = []

    def clear_markers(self):
        """ Clear the markers without deleting them"""
        self._markers_array = []

    def add_markers(self, markers):
        """
        Append more markers at the of the marker array
        """
        if isinstance(markers, list):
            self._markers_array.extend(markers)
        elif isinstance(markers, Marker):
            self._markers_array.append(markers)

    def add_text_marker(self, pose, text, size=0.02, color=None):
        """ Add pose markers to the marker array """
        self._markers_array.append(self.create_text_marker(pose, text, size, color))

    def add_point_markers(self, poses, size=0.02, color=None):
        """ Add point markers to the marker array """
        self._markers_array.extend(self.create_point_markers(poses, size, color))

    def add_line_marker(self, poses, size=0.01, color=None):
        """ Add line marker to the marker array """
        points = [p.pose.position for p in poses]
        self._markers_array.append(self.create_line_marker(points, poses[0].header, size, color))

    def add_box_marker(self, pose, dimensions, color=None):
        """ Add box marker to the marker array """
        self._markers_array.append(self.create_geometry_primitive_marker(pose, dimensions, color, Marker.CUBE))

    def add_disc_marker(self, pose, dimensions, color=None):
        """ Add disc marker to the marker array """
        self._markers_array.append(self.create_geometry_primitive_marker(pose, dimensions, color, Marker.CYLINDER))

    @classmethod
    def rand_color(cls, alpha=1.0):
        """ Create color msg from random RGB values. Alpha defaults to 1.0 """
        return ColorRGBA(random(), random(), random(), alpha)

    @classmethod
    def make_color(cls, rgba: tuple):
        """ Create color msg from a tuple containing RGB[A] values. Alpha defaults to 1.0 """
        if not rgba:  # default to non invasive blue
            return ColorRGBA(0, 0, 1, 0.9)
        if isinstance(rgba, ColorRGBA):
            return rgba
        if len(rgba) == 3:
            return ColorRGBA(*rgba, 1.0)
        if len(rgba) == 4:
            return ColorRGBA(*rgba)
        raise ValueError("Color tuple has to be of len 3 or 4!")

    @classmethod
    def named_color(cls, color_name, alpha=1.0):
        """ Create color msg for some basic color names. Alpha defaults to 1.0 """
        if not hasattr(cls, 'color_map'):
            cls.color_map = {
                "red": ColorRGBA(r=1.0, g=0.0, b=0.0, a=alpha),
                "blue": ColorRGBA(r=0.0, g=0.0, b=1.0, a=alpha),
                "green": ColorRGBA(r=0.0, g=1.0, b=0.0, a=alpha),
                "yellow": ColorRGBA(r=1.0, g=1.0, b=0.0, a=alpha),
                "orange": ColorRGBA(r=1.0, g=0.65, b=0.0, a=alpha),
                "white": ColorRGBA(r=1.0, g=1.0, b=1.0, a=alpha),
                "gray": ColorRGBA(r=0.5, g=0.5, b=0.5, a=alpha),
                "beige": ColorRGBA(r=0.96, g=0.96, b=0.86, a=alpha),
                "cyan": ColorRGBA(r=0.0, g=1.0, b=1.0, a=alpha),
            }

        return cls.color_map[color_name]

    @classmethod
    def create_mesh_marker(cls, pose, mesh_file, scale=None, color=None, namespace='mesh_marker'):
        """ Create a mesh visualization marker object"""
        marker = Marker()
        marker.type = Marker.MESH_RESOURCE
        marker.mesh_resource = mesh_file
        marker.ns = namespace
        marker.scale.x = scale[0] if scale else 1.0
        marker.scale.y = scale[1] if scale else 1.0
        marker.scale.z = scale[2] if scale else 1.0
        marker.color = cls.make_color(color)
        marker.action = Marker.ADD
        marker.header = pose.header
        marker.pose = pose.pose
        return marker

    @classmethod
    def create_geometry_primitive_marker(cls, pose, dimensions, color=None, shape=Marker.CUBE, namespace='gp_marker'):
        """ Create a geometry primitive visualization marker object: CUBE, SPHERE or CYLINDER """
        marker = Marker()
        marker.type = shape
        marker.ns = namespace
        while len(dimensions) < 3:
            dimensions.append(0)
        marker.scale.x = max(dimensions[0], 0.001)
        marker.scale.y = max(dimensions[1], 0.001)
        marker.scale.z = max(dimensions[2], 0.001)
        marker.color = cls.make_color(color)
        marker.action = Marker.ADD
        marker.header = pose.header
        marker.pose = pose.pose
        return marker

    @classmethod
    def create_line_marker(cls, points, header, size=0.01, color=None, namespace='line_strip'):
        """ Create a line strip marker """
        marker = Marker()
        marker.type = Marker.LINE_STRIP
        marker.ns = namespace
        marker.pose.orientation.w = 1.0
        marker.scale.x = size
        marker.color = cls.make_color(color)
        marker.points = points
        marker.action = Marker.ADD
        marker.header = header
        return marker

    @classmethod
    def create_point_marker(cls, pose, size=0.01, color=None, namespace='point_marker'):
        """ Create a point marker from pose input """
        marker = Marker()
        marker.ns = namespace
        marker.type = Marker.SPHERE
        marker.header = pose.header
        marker.pose = pose.pose
        marker.scale.x = size
        marker.scale.y = size
        marker.scale.z = size
        marker.color = cls.make_color(color)
        marker.action = Marker.ADD
        return marker

    @classmethod
    def create_text_marker(cls, pose_st, text, size=0.1, color=None, namespace='text_marker'):
        """ Create a text marker
        Params:
            pose_st (PoseStamped): position of the marker
            text (String): the text to be shown
            size (float): size of the text
        Returns:
            Marker
        """
        marker = Marker()
        marker.header = pose_st.header
        marker.pose = pose_st.pose
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.ns = namespace
        marker.text = text
        marker.color = cls.make_color(color)
        marker.scale.z = size
        return marker

    @classmethod
    def create_collision_obj_markers(cls, col_obj, color=None, namespace='collision_object'):
        """ Create markers for all the primitives of a collision object
        Params:
            col_obj (CollsionObject): collision object to show
        Returns:
            [Marker]: list of markers, one for each primitive
        """
        markers = []
        if len(col_obj.primitives) < len(col_obj.primitive_poses):
            return []
        for i, prim in col_obj.primitives:
            if prim.type != 1:  # only box support
                continue
            p_st = PoseStamped(header=col_obj.header, pose=col_obj.primitive_poses[i])
            markers.append(cls.create_geometry_primitive_marker(p_st, prim.dimensions, color, namespace=namespace))
        return markers

    @classmethod
    def create_point_markers(cls, poses, size=0.01, color=None, namespace='point_marker'):
        """
        create point markers from a list of poses
        Params:
            poses ([PoseStamped]): list of poses
            size (float): size of the markers in meter
        Returns:
            [Marker]
        """
        color = cls.make_color(color)
        markers = list()
        for pose in poses:
            marker = Marker()
            marker.ns = namespace
            marker.type = Marker.SPHERE
            marker.header = pose.header
            marker.pose = pose.pose
            marker.scale.x = size
            marker.scale.y = size
            marker.scale.z = size
            marker.color = color
            marker.action = Marker.ADD
            markers.append(marker)
        return markers

    @classmethod
    def create_cube_list(cls, ref_pose, poses, colors, size=0.01, namespace='cube_list'):
        """
        Creates a cube list referenced to a pose
        Args:
        ref_pose (PoseStamped): This is used for the header and the ref_points
        points (list): List of Point objects with the positions relative to ref_pose
        colors (list): The colors for the markers
        size (float): size of the cubes in meters
        """
        marker = Marker()
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.ns = namespace
        marker.header = ref_pose.header
        marker.pose = ref_pose.pose
        marker.points = poses
        marker.scale.x = size
        marker.scale.y = size
        marker.scale.z = size
        if isinstance(colors[0], list):
            # colors defined for every element
            marker.colors = [cls.make_color(color) for color in colors]
        else:
            marker.color = cls.make_color(colors)
        return marker

    @classmethod
    def create_point_list(cls, ref_pose, points, colors, size=0.01, namespace='point_list'):
        """
        Creates a point list referenced to a pose
        Args:
        ref_pose (PoseStamped): This is used for the header and the ref_points
        points (list): List of Point objects with the positions relative to ref_pose
        colors (list): The colors for the points
        size (float): size of the points in meters
        """
        marker = Marker()
        marker.type = Marker.POINTS
        marker.header = ref_pose.header
        marker.pose = ref_pose.pose
        marker.ns = namespace
        marker.action = Marker.ADD
        marker.scale.x = size
        marker.scale.y = size
        if isinstance(colors[0], list):
            # colors defined for every element
            marker.colors = [cls.make_color(color) for color in colors]
        else:
            marker.color = cls.make_color(colors)
        marker.points = points
        return marker

    @classmethod
    def create_overlay_text(cls, offset_from_top, color, text, text_size):
        msg = OverlayText()
        msg.action = OverlayText.ADD
        msg.width = 1000
        msg.height = 25
        msg.left = 5
        msg.top = offset_from_top
        msg.fg_color = cls.make_color(color)
        msg.text = text
        msg.text_size = text_size
        return msg
