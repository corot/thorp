import smach

from thorp_toolkit.geometry import translate_pose


class TranslatePose(smach.State):
    """
    Apply a displacement to a geometry_msgs pose along a given axis (x, y or z). Returns 'succeeded' in any case.
    """
    def __init__(self, delta, axis):
        super(TranslatePose, self).__init__(outcomes=['succeeded'],
                                            input_keys=['pose'],
                                            output_keys=['pose'])
        self.delta = delta
        self.axis = axis

    def execute(self, ud):
        translate_pose(ud['pose'], self.delta, self.axis)
        return 'succeeded'
