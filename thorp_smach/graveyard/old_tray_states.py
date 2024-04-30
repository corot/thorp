class NextPoseOnTray(smach.State):
    """
    Calculate the next pose where to put an object on the tray.
    """

    def __init__(self, tray_link):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'tray_full'],
                             output_keys=['pose_on_tray'])
        self.tray_slot = rospy.get_param('~tray_slot')
        self.tray_side_x = rospy.get_param('~tray_side_x')
        self.tray_side_y = rospy.get_param('~tray_side_y')
        self.tray_link = tray_link
        self.tray_full = False
        self.slots_x = int(self.tray_side_x / self.tray_slot + 0.1)  # avoid float division pitfall
        self.slots_y = int(self.tray_side_y / self.tray_slot + 0.1)  # until I switch to Python3
        self.offset_x = 0.0 if self.slots_x % 2 else self.tray_slot / 2.0
        self.offset_y = 0.0 if self.slots_y % 2 else self.tray_slot / 2.0
        self.next_x = 0
        self.next_y = 0

        # visualize place poses (for debugging)
        points = []
        for _ in range(self.slots_x * self.slots_y):
            points.append(self._next_pose(0.01).pose.position)
        Visualization().add_markers(Visualization().create_point_list(create_2d_pose(0, 0, 0, self.tray_link),
                                                                      points, [1, 0, 0, 1]))  # solid red points
        Visualization().publish_markers()
        self.tray_full = False

    def execute(self, ud):
        if self.tray_full:
            return 'tray_full'

        # place objects 3cm above the tray, so they fall into position
        ud['pose_on_tray'] = self._next_pose(rospy.get_param('~placing_height_on_tray'))
        return 'succeeded'

    def _next_pose(self, z):
        # Get next empty location coordinates
        x = (self.next_x - self.slots_x/2) * self.tray_slot + self.offset_x
        y = (self.next_y - self.slots_y/2) * self.tray_slot + self.offset_y
        self.next_x = (self.next_x + 1) % self.slots_x
        if self.next_x == 0:
            self.next_y = (self.next_y + 1) % self.slots_y
            if self.next_x == self.next_y == 0:
                self.tray_full = True
        return create_3d_pose(x, y, z, 0, 0, 0, self.tray_link)


class RemoveObject(smach.State):
    """
    Remove collision object from the planning scene
    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded'],
                             input_keys=['object_name'])

    def execute(self, ud):
        rospy.loginfo("Removing object '%s' from planning scene", ud['object_name'])
        PlanningScene().remove_obj(ud['object_name'])
        return 'succeeded'


class ClearPlanningScene(smach.State):
    """
    Clear the planning scene, optionally sparing the tray and its content
    """

    def __init__(self, keep_tray=True):
        smach.State.__init__(self,
                             outcomes=['succeeded'])
        self.keep_tray = keep_tray

    def execute(self, ud):
        rospy.loginfo("Clearing planning scene")
        PlanningScene().remove_all(self.keep_tray)
        return 'succeeded'


class DisplaceObject(smach.State):
    """
    Displace a collision object in the planning scene
    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded'],
                             input_keys=['object', 'new_pose'])

    def execute(self, ud):
        rospy.loginfo("Object '%s' pose readjusted to %s", ud['object'].id, pose2d2str(ud['new_pose']))
        PlanningScene().displace_obj(ud['object'].id, ud['new_pose'])
        return 'succeeded'


class MoveObjToTray(smach.State):
    """
    Move a collision object to the tray. The input pose z-coordinate is at tray's bottom;
    we need to add half the object size, so it appears at the right height on planning scene.
    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded'],
                             input_keys=['object', 'pose_on_tray'])

    def execute(self, ud):
        pose_on_tray = ud['pose_on_tray']
        pose_on_tray.pose.position.z += get_size_from_co(ud['object'])[2] / 2.0
        PlanningScene().move_obj_to_tray(ud['object'].id, pose_on_tray)
        rospy.loginfo("Object '%s' moved to tray at %s", ud['object'].id, pose2d2str(pose_on_tray))
        return 'succeeded'
