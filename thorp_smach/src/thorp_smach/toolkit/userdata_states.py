import rospy
import smach


class UDHasKey(smach.State):
    """
    Check if our ud contains a given key. Returns
    - 'true' if present
    - 'false' otherwise
    """

    def __init__(self, key):
        super(UDHasKey, self).__init__(outcomes=['true', 'false'],
                                       input_keys=[key])
        self.key = key

    def execute(self, ud):
        return 'true' if self.key in ud else 'false'


class UDIfKey(smach.State):
    """
    Check if our ud contains a given key and its content evaluates to True. Returns
    - 'true' if present and True
    - 'false' otherwise
    """

    def __init__(self, key):
        super(UDIfKey, self).__init__(outcomes=['true', 'false'],
                                      input_keys=[key])
        self.key = key

    def execute(self, ud):
        return 'true' if self.key in ud and ud[self.key] else 'false'


class UDSetToNone(smach.State):
    """
    Set a ud key to None, if it exists. Returns
    - 'succeeded' if the key exists
    - 'aborted' otherwise
    """

    def __init__(self, key):
        super(UDSetToNone, self).__init__(outcomes=['succeeded', 'aborted'],
                                          input_keys=[key],
                                          output_keys=[key])
        self.key = key

    def execute(self, ud):
        if self.key in ud:
            ud[self.key] = None
            return 'succeeded'
        rospy.logerr("Trying to set o None unavailable key '%s'", self.key)
        return 'aborted'


class UDInsertInList(smach.State):
    """
    Insert an element in a list at a given position.
    Both 'list' and 'element' are provided as ud keys. Returns
    - 'succeeded' if succeeded
    - 'aborted' otherwise
    """

    def __init__(self, position):
        super(UDInsertInList, self).__init__(outcomes=['succeeded', 'aborted'],
                                             input_keys=['list', 'element'],
                                             output_keys=['list'])
        self.position = position

    def execute(self, ud):
        try:
            ud['list'].insert(self.position, ud['element'])
            return 'succeeded'
        except IndexError as err:
            rospy.logerr("Position out of bounds (%d >= %d)", self.position, len(ud['list']))
            return 'aborted'
