import struct
import random

import rospy
import smach
import smach_ros

import nav_msgs.msg as nav_msgs
import sensor_msgs.msg as sensor_msgs
import geometry_msgs.msg as geometry_msgs
import ipa_building_msgs.msg as ipa_building_msgs

from thorp_toolkit.geometry import TF2, to_pose2d, create_2d_pose

from common_states import UDHasKey
from navigation_states import GetRobotPose, GoToPose, PosesAsPath, ExeSparsePath

import config as cfg


class SegmentRooms(smach_ros.SimpleActionState):
    def __init__(self):
        super(SegmentRooms, self).__init__('exploration/room_segmentation_server',
                                           ipa_building_msgs.MapSegmentationAction,
                                           goal_cb=self.make_goal,
                                           result_cb=self.result_cb,
                                           result_slots=['segmented_map',
                                                         'room_information_in_meter', 'room_information_in_pixel'],
                                           output_keys=['map_image', 'map_origin', 'map_resolution', 'robot_radius'])
        self.img_pub = rospy.Publisher('exploration/room_segmentation_img', sensor_msgs.Image, queue_size=1, latch=True)

    def make_goal(self, ud, goal):
        # Listen for current map
        occ_grid_map = rospy.wait_for_message('/map', nav_msgs.OccupancyGrid)
        goal.input_map = sensor_msgs.Image()
        goal.input_map.header = occ_grid_map.header
        goal.input_map.height = occ_grid_map.info.height
        goal.input_map.width = occ_grid_map.info.width
        goal.input_map.step = occ_grid_map.info.width
        goal.input_map.encoding = 'mono8'
        # Convert map into a black and white 8bit single-channel image (format 8UC1), which is 0 (black)
        # for obstacles and unknown space, and 255 (white) for free space
        for i in range(occ_grid_map.info.height):
            for j in range(occ_grid_map.info.width):
                pixel_value = b'\x00' if occ_grid_map.data[i * occ_grid_map.info.width + j] in [-1, 100] else b'\xFF'
                goal.input_map.data += pixel_value
        # ALTERNATIVE: read image from file
        # import cv2
        # from cv_bridge import CvBridge
        # bridge = CvBridge()
        # cv_image = cv2.imread('/home/jorge/catkin_ws/thorp/src/thorp/thorp_simulation/worlds/maps/cat_house.png',
        #                       cv2.IMREAD_UNCHANGED)
        # for i in range(cv_image.shape[0]):
        #     for j in range(cv_image.shape[1]):
        #         if cv_image[i][j] < 250:
        #             cv_image[i][j] = 0
        #         else:
        #             cv_image[i][j] = 255
        # goal.input_map = bridge.cv2_to_imgmsg(cv_image, encoding='mono8')
        rospy.Publisher('/exploration/img', sensor_msgs.Image, queue_size=1, latch=True).publish(goal.input_map)
        goal.map_origin = occ_grid_map.info.origin
        goal.map_resolution = occ_grid_map.info.resolution
        goal.return_format_in_meter = True
        goal.return_format_in_pixel = True
        goal.robot_radius = rospy.get_param('/move_base_flex/global_costmap/robot_radius', 0.18)
        # those values are also needed by PlanRoomSequence and PlanRoomExploration, so share them as output keys
        # the segmented map comes with its own origin and resolution, but are the same as for the input map
        ud['map_image'] = goal.input_map
        ud['map_origin'] = goal.map_origin
        ud['map_resolution'] = goal.map_resolution
        ud['robot_radius'] = goal.robot_radius

    def result_cb(self, ud, status, result):
        # make a random color rooms map to show in RViz
        rooms_color_map = sensor_msgs.Image()
        rooms_color_map.header = result.segmented_map.header
        rooms_color_map.width = result.segmented_map.width
        rooms_color_map.height = result.segmented_map.height
        rooms_color_map.step = rooms_color_map.width * 3
        rooms_color_map.encoding = 'rgb8'
        for i in range(result.segmented_map.height):
            for j in range(result.segmented_map.width):
                idx = i * result.segmented_map.width * 4 + j * 4
                val = result.segmented_map.data[idx:idx + 4]
                room = struct.unpack('<BBBB', val)[0]
                if room > 0:
                    random.seed(room)
                    r, g, b = random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)
                    rooms_color_map.data += struct.pack('<BBB', r, g, b)
                else:
                    rooms_color_map.data += struct.pack('<BBB', 0, 0, 0)
        self.img_pub.publish(rooms_color_map)


class PlanRoomSequence(smach_ros.SimpleActionState):
    """ Plan rooms visit sequence """
    def __init__(self):
        super(PlanRoomSequence, self).__init__('exploration/room_sequence_planning_server',
                                               ipa_building_msgs.FindRoomSequenceWithCheckpointsAction,
                                               goal_slots=['input_map', 'map_origin', 'map_resolution',
                                                           'robot_radius', 'robot_start_coordinate',
                                                           'room_information_in_pixel'],
                                               result_cb=self.result_cb,
                                               output_keys=['room_sequence'])
        self.img_pub = rospy.Publisher('exploration/room_sequence_img', sensor_msgs.Image, queue_size=1, latch=True)

    def result_cb(self, ud, status, result):
        # room numbers start with 1, so we get them with index + 1
        ud['room_sequence'] = [ri + 1 for ri in result.checkpoints[0].room_indices]
        self.img_pub.publish(result.sequence_map)


class PlanRoomExploration(smach_ros.SimpleActionState):
    def __init__(self):
        super(PlanRoomExploration, self).__init__('exploration/room_exploration_server',
                                                  ipa_building_msgs.RoomExplorationAction,
                                                  goal_cb=self.make_goal,
                                                  goal_slots=['map_resolution', 'map_origin', 'robot_radius'],
                                                  result_slots=['coverage_path', 'coverage_path_pose_stamped'],
                                                  input_keys=['map_image', 'map_resolution',
                                                              'segmented_map', 'robot_pose',
                                                              'room_number', 'room_information_in_meter'],
                                                  output_keys=['start_pose'])
        self.start_pub = rospy.Publisher('/exploration/starting_point', geometry_msgs.PointStamped, queue_size=1)
        self.fov_pub = rospy.Publisher('/exploration/camera_fov', geometry_msgs.PolygonStamped, queue_size=1)
        self.fov_pub_timer = None

    def make_goal(self, ud, goal):
        """ Create a goal for the action
        """
        room_number = ud['room_number']
        segmented_map = ud['segmented_map']

        # We need an image containing only the room to explore, so we create an empty image with the same
        # properties as the original map, set to white (\xFF) all pixels that correspond to the given room
        # number in the segmented map, and set as black (\x00) te rest
        goal.input_map = ud['map_image']
        goal.input_map.data = b''
        # segmented_map encoding is 32SC1, so 4 bytes, though just the first byte is enough up to 255 rooms
        for i in range(segmented_map.height):
            for j in range(segmented_map.width):
                idx = i * segmented_map.step + j * 4
                val = segmented_map.data[idx:idx + 4]
                pixel = struct.unpack('<BBBB', val)[0]
                if pixel == room_number:
                    goal.input_map.data += b'\xFF'
                else:
                    goal.input_map.data += b'\x00'

        fov_points = rospy.get_param('/exploration/room_exploration_server/field_of_view_points')
        goal.field_of_view_origin = TF2().transform_pose(None, 'kinect_rgb_frame', 'base_footprint').pose.position
        goal.field_of_view = [geometry_msgs.Point32(*pt) for pt in fov_points]
        goal.planning_mode = 2  # plan a path for coverage with the robot's field of view
        goal.starting_position = to_pose2d(ud['robot_pose'])  # we need to convert to Pose2D msg

        # use room center as starting point; theta is ignored by room exploration
        room_info = ud['room_information_in_meter'][room_number - 1]
        start_point = geometry_msgs.PointStamped()
        start_point.header.frame_id = 'map'
        start_point.point = room_info.room_center
        goal.starting_position.x = room_info.room_center.x
        goal.starting_position.y = room_info.room_center.y
        goal.starting_position.theta = 0.0  # it's ignored
        self.start_pub.publish(start_point)
        # provide the starting pose so we can move there before starting exploring
        ud['start_pose'] = create_2d_pose(room_info.room_center.x, room_info.room_center.y, 0.0, 'map')
        # IDEA: can use room_info.room_min_max to avoid points colliding with the walls

        # visualize explore map and fov on RViz
        fov = geometry_msgs.PolygonStamped()
        fov.header.frame_id = 'kinect_rgb_frame'
        fov.polygon.points = goal.field_of_view
        rospy.Publisher('/exploration/img', sensor_msgs.Image, queue_size=1, latch=True).publish(goal.input_map)
        if not self.fov_pub_timer:
            self.fov_pub_timer = rospy.Timer(rospy.Duration(0.1), lambda e: self.fov_pub.publish(fov))


def ExploreHouse():
    """
    Explore house SM:
     - segment map into rooms and plan visit sequence
     - iterate over all rooms and explore following the planned sequence
    """
    # explore a single room
    explore_1_room_sm = smach.Sequence(outcomes=['succeeded', 'aborted', 'preempted'],
                                       connector_outcome='succeeded',
                                       input_keys=['map_image', 'map_resolution', 'map_origin', 'robot_radius',
                                                   'segmented_map', 'room_number', 'room_information_in_meter'])
    with explore_1_room_sm:
        smach.Sequence.add('GET_ROBOT_POSE', GetRobotPose())
        smach.Sequence.add('PLAN_ROOM_EXPL', PlanRoomExploration())
        smach.Sequence.add('POSES_AS_PATH', PosesAsPath(),
                           remapping={'poses': 'coverage_path_pose_stamped'})
        smach.Sequence.add('GOTO_START_POSE', GoToPose(dist_tolerance=cfg.LOOSE_DIST_TOLERANCE,   # just close enough
                                                       angle_tolerance=cfg.INF_ANGLE_TOLERANCE),  # ignore orientation
                           remapping={'target_pose': 'start_pose'})
        smach.Sequence.add('TRAVERSE_POSES', ExeSparsePath())

    # iterate over all rooms and explore following the planned sequence
    explore_house_it = smach.Iterator(outcomes=['succeeded', 'preempted', 'aborted'],
                                      input_keys=['map_image', 'map_resolution', 'map_origin', 'robot_radius',
                                                  'segmented_map', 'room_sequence', 'room_information_in_meter'],
                                      output_keys=[],
                                      it=lambda: sm.userdata.room_sequence,  # must be a lambda because we destroy the list  TODO  ehhh???
                                      it_label='room_number',
                                      exhausted_outcome='succeeded')
    with explore_house_it:
        smach.Iterator.set_contained_state('EXPLORE_1_ROOM', explore_1_room_sm, loop_outcomes=['succeeded', 'aborted'])

    # Full SM: segment map, plan rooms visit sequence and explore each room in turn
    sm = smach.StateMachine(outcomes=['succeeded',
                                      'aborted',
                                      'preempted'])
    with sm:
        smach.StateMachine.add('HAVE_SEGMENTED_MAP', UDHasKey('segmented_map'),
                               transitions={'true': 'GET_ROBOT_POSE',
                                            'false': 'SEGMENT_ROOMS'})
        smach.StateMachine.add('SEGMENT_ROOMS', SegmentRooms(),
                               transitions={'succeeded': 'GET_ROBOT_POSE'})
        smach.StateMachine.add('GET_ROBOT_POSE', GetRobotPose(),
                               transitions={'succeeded': 'PLAN_ROOM_SEQ'})
        smach.StateMachine.add('PLAN_ROOM_SEQ', PlanRoomSequence(),
                               transitions={'succeeded': 'EXPLORE_HOUSE'},
                               remapping={'robot_start_coordinate': 'robot_pose',
                                          'input_map': 'map_image'})
        smach.StateMachine.add('EXPLORE_HOUSE', explore_house_it)
    return sm
