"""
The one thing that actually touches the robot: a RunSubtree goal per capability call.
"""

import json
import time

from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient

from thorp_msgs.action import RunSubtree

from . import errors

DEFAULT_ACTION = '/bt_server/run_subtree'


def wait(future, timeout):
    """
    The future's result, or None if it isn't done after timeout seconds. Tools run on the agent's thread, while an
    executor spins the node on another, so blocking here doesn't stop the future from completing.
    """
    deadline = time.monotonic() + timeout
    while not future.done():
        if time.monotonic() > deadline:
            return None
        time.sleep(0.02)
    return future.result()


class SubtreeRunner(object):
    def __init__(self, node, action_name=DEFAULT_ACTION, connect_timeout=30.0, dry_run=False, world=None):
        self.node = node
        self.dry_run = dry_run
        self.action_name = action_name
        self.error_names = errors.ros_table()
        # read after every run, so the agent is told where things stand rather than inferring it from what it has
        # done, and without spending a call to ask
        self.world = world
        if dry_run:
            self.client = None
            return
        self.client = ActionClient(node, RunSubtree, action_name)
        if not self.client.wait_for_server(timeout_sec=connect_timeout):
            raise RuntimeError('no RunSubtree server on {} after {}s'.format(action_name, connect_timeout))

    def run(self, subtree, inputs=None, output_keys=None, timeout=300.0):
        """
        Send one goal and return what the tree produced, as a dict.

        Returns rather than raises on a robot-level failure. A model that gets an exception learns its tool is broken;
        one that gets {"succeeded": false, "error": -200} can read the code, decide the object was not found and try
        something else, which is the whole point of putting it in charge.
        """
        goal = RunSubtree.Goal()
        goal.subtree = subtree
        goal.json = json.dumps(inputs or {})
        goal.output_keys = sorted(output_keys or [])

        if self.dry_run:
            self.node.get_logger().info('[dry run] {} {}'.format(subtree, goal.json))
            return {'succeeded': True, 'dry_run': True, 'outputs': {}}

        handle = wait(self.client.send_goal_async(goal), 10.0)
        if handle is None or not handle.accepted:
            return self.observed({'succeeded': False, 'refused': True, 'error': 'bt_server did not accept the goal'})
        response = wait(handle.get_result_async(), timeout)
        if response is None:
            wait(handle.cancel_goal_async(), 5.0)
            return self.observed({'succeeded': False,
                                  'error': 'no result after {}s; the goal was canceled'.format(timeout)})

        result = response.result
        if response.status == GoalStatus.STATUS_ABORTED:
            # bt_server aborted the goal: unknown tree, malformed json, a missing input or a node that threw. Its json
            # says which, so hand that straight back.
            detail = json.loads(result.json) if result.json else {}
            return self.observed({'succeeded': False, 'refused': True,
                                  'error': detail.get('error', 'the goal was refused')})

        outputs = errors.annotate(json.loads(result.json) if result.json else {}, self.error_names)
        return self.observed({'succeeded': bool(result.success),
                              'state': _STATE_NAMES.get(response.status, str(response.status)),
                              'outputs': outputs})

    def observed(self, result):
        """Adds what the robot looks like now. A failed run is when it matters most."""
        if self.world:
            result['observed'] = self.world.observe()
        return result


_STATE_NAMES = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_CANCELED: 'CANCELED',
                GoalStatus.STATUS_ABORTED: 'ABORTED'}
