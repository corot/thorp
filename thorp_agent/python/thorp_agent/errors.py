"""
Names for the error codes capabilities return, so the agent reads INVALID_TARGET_POSE rather than guessing what -210
means. Built from the message definitions, so a code added to ThorpError.msg gets its name without touching this.
"""

import importlib

# Where the codes come from, first match winning a clash: Thorp's own, Nav2's servers, then the MoveIt codes Thorp's
# are an extension of. Thorp's and MoveIt's are negative and Nav2's positive, so in practice only 0 and 1 clash:
# Nav2's NONE has to come before MoveIt's UNDEFINED, as capabilities report 0 when they worked.
SOURCES = (('thorp_msgs.msg', ('ThorpError',)),
           ('nav2_msgs.action', ('NavigateToPose.Result', 'NavigateThroughPoses.Result', 'FollowPath.Result',
                                 'ComputePathToPose.Result', 'ComputePathThroughPoses.Result', 'SmoothPath.Result',
                                 'Spin.Result', 'BackUp.Result', 'FollowObject.Result')),
           ('moveit_msgs.msg', ('MoveItErrorCodes',)))


def table(*message_classes):
    """code -> name, from the integer constants of each message class"""
    names = {}
    for cls in message_classes:
        for name in dir(cls):
            value = getattr(cls, name, None)
            if name.isupper() and isinstance(value, int) and not isinstance(value, bool):
                names.setdefault(value, name)
    return names


def _resolve(module, dotted):
    for part in dotted.split('.'):
        module = getattr(module, part, None)
        if module is None:
            return None
    return module


def ros_table():
    classes = []
    for module_name, class_names in SOURCES:
        try:
            module = importlib.import_module(module_name)
        except ImportError:
            continue
        classes += [cls for cls in (_resolve(module, name) for name in class_names) if cls is not None]
    return table(*classes)


def annotate(outputs, names):
    """Adds <key>_name beside each error output with a known code, and returns outputs"""
    for key, value in list(outputs.items()):
        if (key == 'error' or key.endswith('_error')) and isinstance(value, int) and value in names:
            outputs[key + '_name'] = names[value]
    return outputs
