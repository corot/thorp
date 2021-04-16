import smach

from thorp_toolkit.semantic_layer import SemanticLayer


class AddToCostmap(smach.State):
    """ Add an object to the semantic layer of one or both costmaps """

    def __init__(self):
        super(AddToCostmap, self).__init__(outcomes=['succeeded'],
                                           input_keys=['name', 'type', 'pose', 'size', 'costmap'])

    def execute(self, ud):
        SemanticLayer().add_object(ud['name'], ud['type'], ud['pose'], ud['size'], ud['costmap'])
        return 'succeeded'


class DelFromCostmap(smach.State):
    """ Remove an object from the semantic layer of one or both costmaps """

    def __init__(self):
        super(DelFromCostmap, self).__init__(outcomes=['succeeded'],
                                             input_keys=['name', 'type', 'costmap'])

    def execute(self, ud):
        SemanticLayer().remove_object(ud['name'], ud['type'], ud['costmap'])
        return 'succeeded'


class TableAsObstacle(smach.State):
    """
    Mark table area as an obstacle on both local and global costmaps, so robot doesn't collide with the invisible eaves.
    """

    def __init__(self):
        super(TableAsObstacle, self).__init__(outcomes=['succeeded'],
                                              input_keys=['table', 'pose'])

    def execute(self, ud):
        name, width, length = ud['table'].name, ud['table'].width, ud['table'].depth
        SemanticLayer().add_object(name, 'obstacle', ud['pose'], (width, length), 'both')
        return 'succeeded'


class ClearTableWay(smach.State):
    """
    Clear an area on local costmap so the controller can approach the table
    """

    def __init__(self):
        super(ClearTableWay, self).__init__(outcomes=['succeeded'],
                                            input_keys=['table', 'pose'])

    def execute(self, ud):
        obj_name = ud['table'].name + ' approach'
        SemanticLayer().add_object(obj_name, 'free_space', ud['pose'], [1.0, 0.5], 'local')
        return 'succeeded'


class RestoreTableWay(smach.State):
    """
    restore the area cleared to approach the table, so we don't collide with it after detaching
    """

    def __init__(self):
        super(RestoreTableWay, self).__init__(outcomes=['succeeded'],
                                              input_keys=['table'])

    def execute(self, ud):
        obj_name = ud['table'].name + ' approach'
        SemanticLayer().remove_object(obj_name, 'free_space', 'local')
        return 'succeeded'
