"""
Shared fixtures for the bt_server test suite.

Two tiers live side by side here:

  test_bt_server.py    the action's own contract, against trees that need nothing but a
                       roscore and bt_server. Seconds, deterministic, CI-able.
  test_capabilities.py every tree in config/capabilities.yaml, which mostly means a running
                       robot stack. Minutes, and only as deterministic as the robot.

Both need a roscore and bt_server up; neither starts them, deliberately. Bringing up Gazebo
per test file would put a couple of minutes between you and every run, so the stack is
somebody else's job and these just connect to it.

    cd /catkin_ws/src/thorp/thorp_bt_cpp && python3 -m pytest test/ -v
    cd /catkin_ws/src/thorp/thorp_bt_cpp && python3 -m pytest test/test_bt_server.py -v
    cd /catkin_ws/src/thorp/thorp_bt_cpp && python3 -m pytest test/ -v -k pickup
    cd /catkin_ws/src/thorp/thorp_bt_cpp && python3 -m pytest test/ -v --stack navigation,manipulation,perception
"""

import json
import os

import pytest
import yaml

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus

from thorp_msgs.msg import RunSubtreeAction, RunSubtreeGoal

STATUS_NAMES = {
    GoalStatus.PENDING: "PENDING",
    GoalStatus.ACTIVE: "ACTIVE",
    GoalStatus.PREEMPTED: "PREEMPTED",
    GoalStatus.SUCCEEDED: "SUCCEEDED",
    GoalStatus.ABORTED: "ABORTED",
    GoalStatus.REJECTED: "REJECTED",
    GoalStatus.RECALLED: "RECALLED",
    GoalStatus.LOST: "LOST",
}


def status_name(state):
    return STATUS_NAMES.get(state, str(state))


def pytest_addoption(parser):
    parser.addoption("--action", default="/run_subtree", help="RunSubtree action name")
    parser.addoption("--param-ns", default="/bt_server",
                     help="bt_server's private namespace, where trees read parameters from")
    parser.addoption("--stack", default="none",
                     help="comma separated list of stacks that are actually running "
                          "(none,navigation,manipulation,perception). Capabilities needing "
                          "anything not listed here are skipped rather than failed.")
    parser.addoption("--capabilities", default=None, help="path to capabilities.yaml")


@pytest.fixture(scope="session")
def available_stacks(request):
    raw = request.config.getoption("--stack")
    return {s.strip() for s in raw.split(",") if s.strip()}


@pytest.fixture(scope="session")
def capabilities(request):
    path = request.config.getoption("--capabilities")
    if not path:
        # works from a source checkout; falls back to the installed share/ directory
        here = os.path.dirname(os.path.abspath(__file__))
        path = os.path.join(here, os.pardir, "config", "capabilities.yaml")
        if not os.path.exists(path):
            import rospkg
            path = os.path.join(rospkg.RosPack().get_path("thorp_bt_cpp"), "config", "capabilities.yaml")
    with open(path) as f:
        return yaml.safe_load(f)["capabilities"]


@pytest.fixture(scope="session")
def ros_node():
    """
    rospy.init_node can only be called once per process, hence session scope. Deliberately not
    autouse: test_capabilities_match_trees.py is pure parsing, and shouldn't need a master
    just because it shares a directory with tests that do. Only `runner` pulls this in.
    """
    rospy.init_node("test_bt_server", anonymous=True, disable_signals=True)


class SubtreeRunner:
    """Thin wrapper over the action client: send a goal, get back parsed json."""

    def __init__(self, action_name, param_ns):
        self.param_ns = param_ns
        self.client = actionlib.SimpleActionClient(action_name, RunSubtreeAction)
        if not self.client.wait_for_server(rospy.Duration(10.0)):
            pytest.fail("action server '{}' not available -- is bt_server_node running?".format(action_name))

    def set_param(self, name, value):
        rospy.set_param("{}/{}".format(self.param_ns.rstrip("/"), name), value)

    def run(self, subtree, inputs=None, output_keys=None, raw_json=None, timeout=30.0, cancel_after=None):
        """Returns (state, parsed_json, result). `raw_json` bypasses json.dumps."""
        goal = RunSubtreeGoal()
        goal.subtree = subtree
        goal.json = raw_json if raw_json is not None else (json.dumps(inputs) if inputs is not None else "")
        goal.output_keys = output_keys or []

        print("\n--> subtree={!r} json={} output_keys={}".format(goal.subtree, goal.json, list(goal.output_keys)))
        self.client.send_goal(goal)

        if cancel_after is not None:
            rospy.sleep(cancel_after)
            print("--> cancelling after {}s".format(cancel_after))
            self.client.cancel_goal()

        if not self.client.wait_for_result(rospy.Duration(timeout)):
            self.client.cancel_goal()
            pytest.fail("subtree '{}' gave no result within {}s".format(subtree, timeout))

        state = self.client.get_state()
        result = self.client.get_result()
        raw = result.json if result else ""
        print("<-- state={} success={} json={}".format(
            status_name(state), result.success if result else "?", raw))

        try:
            parsed = json.loads(raw) if raw else {}
        except ValueError as e:
            pytest.fail("result json is not parseable: {} -- raw was {!r}".format(e, raw))
        return state, parsed, result


@pytest.fixture(scope="session")
def runner(request, ros_node):
    return SubtreeRunner(request.config.getoption("--action"), request.config.getoption("--param-ns"))
