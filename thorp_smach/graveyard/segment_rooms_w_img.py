class SegmentRooms(smach_ros.SimpleActionState):
    def __init__(self):
        super(SegmentRooms, self).__init__('exploration/room_segmentation',
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
        # cv_image = cv2.imread('/home/jorge/catkin_ws/thorp/src/thorp/thorp_simulation/worlds/maps/fun_house.png',
        #                       cv2.IMREAD_UNCHANGED)
        # for i in range(cv_image.shape[0]):
        #     for j in range(cv_image.shape[1]):
        #         if cv_image[i][j] < 250:
        #             cv_image[i][j] = 0
        #         else:
        #             cv_image[i][j] = 255
        # goal.input_map = bridge.cv2_to_imgmsg(cv_image, encoding='mono8')
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

