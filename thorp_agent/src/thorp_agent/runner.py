"""
The one thing that actually touches the robot: a RunSubtree goal per capability call.
"""

import json

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus

from thorp_msgs.msg import RunSubtreeAction, RunSubtreeGoal

from . import errors

DEFAULT_ACTION = "/bt_server/run_subtree"


class SubtreeRunner(object):
    def __init__(self, action_name=DEFAULT_ACTION, connect_timeout=30.0, dry_run=False):
        self.dry_run = dry_run
        self.action_name = action_name
        self.error_names = errors.ros_table()
        if dry_run:
            self.client = None
            return
        self.client = actionlib.SimpleActionClient(action_name, RunSubtreeAction)
        if not self.client.wait_for_server(rospy.Duration(connect_timeout)):
            raise RuntimeError("no RunSubtree server on {} after {}s".format(
                action_name, connect_timeout))

    def run(self, subtree, inputs=None, output_keys=None, timeout=300.0):
        """
        Send one goal and return what the tree produced, as a dict.

        Returns rather than raises on a robot-level failure. A model that gets an exception
        learns its tool is broken; one that gets {"succeeded": false, "error": -200} can read
        the code, decide the object was not found and try something else, which is the whole
        point of putting it in charge.
        """
        goal = RunSubtreeGoal()
        goal.subtree = subtree
        goal.json = json.dumps(inputs or {})
        goal.output_keys = sorted(output_keys or [])

        if self.dry_run:
            rospy.loginfo("[dry run] %s %s", subtree, goal.json)
            return {"succeeded": True, "dry_run": True, "outputs": {}}

        self.client.send_goal(goal)
        if not self.client.wait_for_result(rospy.Duration(timeout)):
            self.client.cancel_goal()
            return {"succeeded": False,
                    "error": "no result after {}s; the goal was canceled".format(timeout)}

        state = self.client.get_state()
        result = self.client.get_result()

        if state == GoalStatus.ABORTED:
            # bt_server refused the goal: unknown tree, malformed json, a missing input. The
            # agent's mistake to fix, and its json says which, so hand that straight back.
            detail = json.loads(result.json) if result and result.json else {}
            return {"succeeded": False, "refused": True,
                    "error": detail.get("error", "the goal was refused")}

        outputs = errors.annotate(json.loads(result.json) if result and result.json else {},
                                  self.error_names)
        return {"succeeded": bool(result and result.success),
                "state": _STATE_NAMES.get(state, str(state)),
                "outputs": outputs}


_STATE_NAMES = {GoalStatus.SUCCEEDED: "SUCCEEDED", GoalStatus.PREEMPTED: "PREEMPTED",
                GoalStatus.ABORTED: "ABORTED", GoalStatus.REJECTED: "REJECTED",
                GoalStatus.RECALLED: "RECALLED", GoalStatus.LOST: "LOST"}
