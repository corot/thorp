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

from std_srvs.srv import Empty

from thorp_msgs.msg import RunSubtreeAction, RunSubtreeGoal
from thorp_msgs.srv import ClearPlanningScene

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
    parser.addoption("--action", default="/bt_server/run_subtree",
                     help="RunSubtree action name; bt_server advertises it in its own "
                          "private namespace, so this follows the node's name")
    parser.addoption("--param-ns", default="/bt_server",
                     help="bt_server's private namespace, where trees read parameters from")
    parser.addoption("--stack", default="none",
                     help="comma separated list of stacks that are actually running "
                          "(none,navigation,manipulation,perception). Capabilities needing "
                          "anything not listed here are skipped rather than failed.")
    parser.addoption("--capabilities", default=None, help="path to capabilities.yaml")
    parser.addoption("--apps", action="store_true",
                     help="also exercise the entries marked `kind: app`. They are skipped by "
                          "default: an app drives the robot until something stops it, or waits "
                          "for a person at the keyboard, so running one inside a test suite "
                          "means a cancel-and-hope rather than a result.")


def _try_service(name, srv_type, **kwargs):
    """
    Call a service if it's there, and shrug if it isn't.

    The same fixture runs whether you brought up a simulator, a real robot or neither, and a
    reset that isn't available is not a reason to fail a test -- it just means there is
    nothing to put back. Anything worse than absence (the call itself failing) is worth
    seeing, so it's printed rather than swallowed.
    """
    try:
        rospy.wait_for_service(name, timeout=2.0)
    except rospy.ROSException:
        return False
    try:
        rospy.ServiceProxy(name, srv_type)(**kwargs)
        return True
    except rospy.ServiceException as e:
        print("--> {} failed: {}".format(name, e))
        return False


def reset_scene():
    """
    Put the world back to how it started.

    Three parts, because none of them is enough alone. /gazebo/reset_world returns every model to
    its spawn pose, so the cubes are back on the table -- but MoveIt doesn't watch gazebo, so
    its planning scene still holds the objects where they used to be, and possibly one still
    attached to the gripper. manipulation/clear_planning_scene empties that, and the next
    detect_objects refills it from what the camera can actually see.

    The sleep is for physics: models dropped back onto the table need a moment to settle
    before a detection of them means anything.

    Caveat worth knowing: reset_world yanks the robot's own model back too, arm joints
    included, while the controllers are running. That's the part of this to keep an eye on.

    A plain function rather than a fixture, deliberately: a fixture runs before the test body,
    which means before the body has decided whether it is going to skip. Resetting the world
    for a test that is about to say "needs navigation running" costs a second and a confusing
    pile of gazebo log lines. The caller resets once it knows it will use the robot.
    """
    # The gripper goes first, and it is the one that matters. An object attached to the gripper
    # in MoveIt's scene survives both of the calls below AND a restart of the test process,
    # since the stack keeps running: the next pickup then reports OBJECT_NOT_FOUND (-200) for
    # an object it believes it is already holding, and says so while writing that object's name
    # into attached_object. This is the service pickup_object's own tree uses to get out of it.
    if not _try_service("/clear_gripper", Empty):
        _try_service("/manipulation/clear_gripper", Empty)
    reset = _try_service("/gazebo/reset_world", Empty)
    _try_service("/manipulation/clear_planning_scene", ClearPlanningScene, keep_tray=False)
    if reset:
        rospy.sleep(1.0)


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
            print("--> canceling after {}s".format(cancel_after))
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
