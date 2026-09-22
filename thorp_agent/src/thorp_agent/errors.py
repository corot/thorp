"""
Names for the error codes capabilities return, so the agent reads INVALID_TARGET_POSE rather
than guessing what -210 means. Built from the message definitions, so a code added to
ThorpError.msg gets its name without touching this.
"""

import importlib

# Where the codes come from, first match winning a clash: Thorp's own, the MoveIt codes Thorp's
# are an extension of, then MBF's for navigation. The first two are negative and MBF's positive,
# so in practice only SUCCESS clashes.
SOURCES = (("thorp_msgs.msg", ("ThorpError",)),
           ("moveit_msgs.msg", ("MoveItErrorCodes",)),
           ("mbf_msgs.msg", ("MoveBaseResult", "ExePathResult", "GetPathResult", "RecoveryResult")))


def table(*message_classes):
    """code -> name, from the integer constants of each message class"""
    names = {}
    for cls in message_classes:
        for name, value in vars(cls).items():
            if name.isupper() and isinstance(value, int) and not isinstance(value, bool):
                names.setdefault(value, name)
    return names


def ros_table():
    classes = []
    for module_name, class_names in SOURCES:
        try:
            module = importlib.import_module(module_name)
        except ImportError:
            continue
        classes += [getattr(module, c) for c in class_names if hasattr(module, c)]
    return table(*classes)


def annotate(outputs, names):
    """Adds <key>_name beside each error output with a known code, and returns outputs"""
    for key, value in list(outputs.items()):
        if (key == "error" or key.endswith("_error")) and isinstance(value, int) and value in names:
            outputs[key + "_name"] = names[value]
    return outputs
