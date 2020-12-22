class MonitorTables(smach.State):
    """
    Look for tables until one is found or we run out of time. Returns:
    - 'succeeded' if a table is seen before timeout (unlimited by default)
    - 'aborted' otherwise
    """

    def __init__(self, timeout=0.0):
        super(MonitorTables, self).__init__(outcomes=['succeeded', 'preempted', 'aborted'],
                                            output_keys=['table', 'table_pose'])
        self.timeout = timeout
        self.detected_table = None
        self.table_event = threading.Condition()
        self.table_sub = rospy.Subscriber("rail_segmentation/segmented_table", rail_msgs.SegmentedObject, self.table_cb)
        self.segment_srv = rospy.ServiceProxy('rail_segmentation/segment_objects', rail_srvs.SegmentObjects)
        self.segment_srv.wait_for_service(30.0)

    def table_cb(self, msg):
        self.detected_table = msg
        self.table_event.acquire()
        self.table_event.notify()
        self.table_event.release()

    def execute(self, ud):
        self.detected_table = None
        start_time = rospy.get_time()
        rate = rospy.Rate(1.0)
        while not self.preempt_requested() and not rospy.is_shutdown():
            self.segment_srv()
            self.table_event.acquire()
            self.table_event.wait(0.1)
            self.table_event.release()
            if self.detected_table:
                pose = geo_msgs.PoseStamped(self.detected_table.point_cloud.header,
                                            geo_msgs.Pose(self.detected_table.center, self.detected_table.orientation))
                pose = TF2().transform_pose(pose, pose.header.frame_id, 'map')
                pose.pose.orientation = quaternion_msg_from_yaw(0)  #  assume tables aligned with x
                                                        # TODO restore once I fix RAIL to provide propper orientation
                width, length = self.detected_table.width, self.detected_table.depth
                table_tf = to_transform(pose, 'table_frame')
                TF2().publish_transform(table_tf)
                # print self.detected_table.point_cloud.header
                # print 'centroid ', self.detected_table.centroid
                # print 'center   ', self.detected_table.center
                # print 'yaw      ', yaw(self.detected_table.orientation)
                # kk=self.detected_table.marker
                ud['table'] = self.detected_table
                ud['table_pose'] = pose
                rospy.loginfo("Detected table of size %.1f x %.1f at %s", width, length, pose2d2str(pose))
                # Add the table contour as an obstacle to global costmap, so we can plan around it
                # We also add an shrinked version to the local costmap so no to impair approaching for picking
                # TODO: detected_table.name is empty; tweak RAIL to provide it or add sequential names here
                # SemanticLayer().add_obstacle(self.detected_table.name, pose, [width, length, 0.0], 'global')
                # SemanticLayer().add_obstacle(self.detected_table.name, pose, [width - 0.2, length - 0.2, 0.0], 'local')    TDDO  a ver q pasa
                return 'succeeded'
            elif self.timeout and rospy.get_time() - start_time > self.timeout:
                rospy.logwarn("No table detected after %g seconds", rospy.get_time() - start_time)
                return 'aborted'
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break
        rospy.logwarn("Detect table has been preempted after %g seconds (timeout %g)",
                      rospy.get_time() - start_time, self.timeout)
        return 'preempted'
