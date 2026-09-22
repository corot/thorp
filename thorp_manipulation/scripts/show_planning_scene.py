#!/usr/bin/env python3

"""
What MoveIt believes is around the robot right now: the collision objects, whatever the
gripper is holding, and how much octomap there is.

RViz shows the same thing, but draws the octomap over the collision objects, so the one
question worth asking when a pick fails -- is the cube still in the scene? -- is the one it
answers worst.

    python3 /catkin_ws/src/thorp/thorp_manipulation/scripts/show_planning_scene.py
    python3 /catkin_ws/src/thorp/thorp_manipulation/scripts/show_planning_scene.py --watch 2
"""

import argparse
import json
import sys

import rospy
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene, GetPlanningSceneRequest

SERVICE = "/get_planning_scene"

COMPONENTS = (PlanningSceneComponents.WORLD_OBJECT_NAMES |
              PlanningSceneComponents.WORLD_OBJECT_GEOMETRY |
              PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS |
              PlanningSceneComponents.OCTOMAP)


def color(obj):
    """The color the detector recorded in type.db; '-' for an object that carries none"""
    try:
        return json.loads(obj.type.db).get("color", "-")
    except (TypeError, ValueError):
        return "-"


def describe(obj):
    shapes = len(obj.primitives) + len(obj.meshes)
    return "  {:<14} {:<11} {:>2} shape{}  at ({: .3f},{: .3f},{: .3f}) in {}".format(
        obj.id, color(obj), shapes, " " if shapes == 1 else "s",
        obj.pose.position.x, obj.pose.position.y, obj.pose.position.z,
        obj.header.frame_id or "?")


def report(scene):
    lines = []

    objects = scene.world.collision_objects
    lines.append("collision objects: {}".format(len(objects)))
    lines += [describe(obj) for obj in objects]

    attached = scene.robot_state.attached_collision_objects
    lines.append("attached: {}".format(len(attached)))
    lines += ["  {} on {}".format(a.object.id, a.link_name) for a in attached]

    # No node count without decoding the octree, but its size answers the question that
    # matters: whether there is an octomap at all, and whether clearing it did anything.
    octomap = scene.world.octomap.octomap
    lines.append("octomap: {} bytes at {} m resolution{}".format(
        len(octomap.data), octomap.resolution or "?",
        "" if octomap.data else "  (empty)"))
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--watch", type=float, metavar="SECONDS",
                        help="keep printing, this many seconds apart")
    args = parser.parse_args()

    rospy.init_node("show_planning_scene", anonymous=True)
    try:
        rospy.wait_for_service(SERVICE, timeout=5.0)
    except rospy.ROSException:
        sys.exit("{} is not there; is move_group running?".format(SERVICE))
    get_scene = rospy.ServiceProxy(SERVICE, GetPlanningScene)

    request = GetPlanningSceneRequest()
    request.components.components = COMPONENTS
    while not rospy.is_shutdown():
        print(report(get_scene(request).scene))
        if not args.watch:
            return
        print("-" * 72)
        rospy.sleep(args.watch)


if __name__ == "__main__":
    main()
