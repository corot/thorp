import smach

from thorp_toolkit.semantic_map import SemanticMap


class TableWasVisited(smach.State):
    """
    Check whether a table has been visited before
    """

    def __init__(self):
        super(TableWasVisited, self).__init__(outcomes=['true', 'false'],
                                              input_keys=['table', 'table_pose'])

    def execute(self, ud):
        intersecting_objs = SemanticMap().objects_at(ud['table_pose'], (ud['table'].depth, ud['table'].width))
        outcome = 'true' if len(intersecting_objs) > 0 else 'false'
        return outcome


class TableMarkVisited(smach.State):
    """
    Mark table as visited, adding it to the semantic map either as a valid table or as junk
    HACK: table detection doesn't provide a unique name, so we provide a sequential name here
    """

    def __init__(self, valid=True):
        super(TableMarkVisited, self).__init__(outcomes=['succeeded'],
                                               input_keys=['table', 'table_pose'],
                                               output_keys=['table'])
        self.valid = valid

    def execute(self, ud):
        obj_name = 'table ' + str(SemanticMap().objects_count('table') + 1) if self.valid else \
                   'junk ' + str(SemanticMap().objects_count('junk') + 1)
        obj_type = 'table' if self.valid else 'junk'
        obj_size = ud['table'].depth, ud['table'].width, ud['table'].height
        SemanticMap().add_object(obj_name, obj_type, ud['table_pose'], obj_size)
        ud['table'].name = obj_name  # give a name to the segmented object
        return 'succeeded'
