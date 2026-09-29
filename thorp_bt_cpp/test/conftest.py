"""
Shared fixtures for the bt_server test suite.

Three tiers live side by side here:

  test_bt_server.py                 the action's own contract, against trees that need nothing but
                                    bt_server. Seconds, deterministic, CI-able.
  test_capabilities.py              every tree in config/capabilities.yaml, which mostly means a
                                    running robot stack. Minutes, and only as deterministic as the robot.
  test_capabilities_match_trees.py  config/capabilities.yaml against the trees; pure parsing.

The first two need bt_server up, and don't start it, deliberately. Bringing up Gazebo per test
file would put a couple of minutes between you and every run, so the stack is somebody else's
job and these just connect to it:

    ros2 run thorp_bt_cpp bt_server_node
    ros2 launch thorp_apps object_manip.launch.py executive:=llm

Then, from the package directory:

    python3 -m pytest test/ -v
    python3 -m pytest test/test_bt_server.py -v
    python3 -m pytest test/ -v -k pickup
    python3 -m pytest test/ -v --stack manipulation,perception --given at_a_table
"""

import json
import os
import re
import subprocess
import time
import xml.etree.ElementTree as ET
from glob import glob

import pytest
import yaml

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient

from geometry_msgs.msg import Pose
from nav2_msgs.srv import ClearEntireCostmap
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from thorp_msgs.action import RunSubtree

STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
    GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
    GoalStatus.STATUS_EXECUTING: "EXECUTING",
    GoalStatus.STATUS_CANCELING: "CANCELING",
    GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
    GoalStatus.STATUS_CANCELED: "CANCELED",
    GoalStatus.STATUS_ABORTED: "ABORTED",
}


def status_name(state):
    return STATUS_NAMES.get(state, str(state))


def installed_trees():
    """
    Tree id -> file name, for the trees installed with thorp_bt_cpp: those ported to ROS 2, that bt_server offers.
    The rest are still ROS 1, and their capabilities are skipped as not ported yet.
    """
    trees = {}
    for path in sorted(glob(os.path.join(get_package_share_directory("thorp_bt_cpp"), "bt", "*.xml"))):
        for tree in ET.parse(path).getroot().findall("BehaviorTree"):
            trees[tree.get("ID")] = os.path.basename(path)
    return trees


def pytest_addoption(parser):
    parser.addoption("--action", default="/bt_server/run_subtree", help="RunSubtree action name")
    parser.addoption("--node", default="/bt_server",
                     help="bt_server's node, whose parameters the trees read")
    parser.addoption("--stack", default="none",
                     help="comma separated list of stacks that are actually running "
                          "(none,navigation,manipulation,perception). Capabilities needing "
                          "anything not listed here are skipped rather than failed.")
    parser.addoption("--given", default="",
                     help="comma separated list of states (see capabilities.yaml) the world "
                          "already provides, e.g. at_a_table on the playground world, where the "
                          "robot starts in front of the table. Setup steps establishing only "
                          "given states are skipped, and so are the stacks they need.")
    parser.addoption("--capabilities", default=None, help="path to capabilities.yaml")
    parser.addoption("--apps", action="store_true",
                     help="also exercise the entries marked `kind: app`. They are skipped by "
                          "default: an app drives the robot until something stops it, or waits "
                          "for a person, so running one inside a test suite means a "
                          "cancel-and-hope rather than a result.")


@pytest.fixture(scope="session")
def available_stacks(request):
    raw = request.config.getoption("--stack")
    return {s.strip() for s in raw.split(",") if s.strip()}


@pytest.fixture(scope="session")
def given_states(request):
    raw = request.config.getoption("--given")
    return {s.strip() for s in raw.split(",") if s.strip()}


def capabilities_path(config):
    path = config.getoption("--capabilities")
    if not path:
        # works from a source checkout; falls back to the installed share/ directory
        here = os.path.dirname(os.path.abspath(__file__))
        path = os.path.join(here, os.pardir, "config", "capabilities.yaml")
        if not os.path.exists(path):
            path = os.path.join(get_package_share_directory("thorp_bt_cpp"), "config", "capabilities.yaml")
    return path


@pytest.fixture(scope="session")
def capabilities(request):
    with open(capabilities_path(request.config)) as f:
        return yaml.safe_load(f)["capabilities"]


@pytest.fixture(scope="session")
def ros_node():
    """
    Session scope, as the ROS context is per process. Deliberately not autouse:
    test_capabilities_match_trees.py is pure parsing, and shouldn't need ROS just because it
    shares a directory with tests that do. Only `runner` pulls this in.
    """
    rclpy.init()
    node = rclpy.create_node("test_bt_server")
    yield node
    node.destroy_node()
    rclpy.try_shutdown()


class SubtreeRunner:
    """Thin wrapper over the action client: send a goal, get back parsed json."""

    def __init__(self, node, action_name, server_node):
        self.node = node
        self.client = ActionClient(node, RunSubtree, action_name)
        if not self.client.wait_for_server(timeout_sec=10.0):
            pytest.fail("action server '{}' not available -- is bt_server_node running?".format(action_name))
        self.parameters = AsyncParameterClient(node, server_node)

    def wait(self, future, timeout):
        """Spin until the future completes; False if it doesn't within the timeout."""
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        return future.done()

    def set_param(self, name, value):
        results = self.parameters.set_parameters([Parameter(name, value=value)])
        assert self.wait(results, 5.0), "setting parameter {} timed out".format(name)
        assert results.result().results[0].successful, results.result().results[0].reason

    def run(self, subtree, inputs=None, output_keys=None, raw_json=None, timeout=30.0, cancel_after=None):
        """Returns (state, parsed_json, result). `raw_json` bypasses json.dumps."""
        goal = RunSubtree.Goal()
        goal.subtree = subtree
        goal.json = raw_json if raw_json is not None else (json.dumps(inputs) if inputs is not None else "")
        goal.output_keys = output_keys or []

        print("\n--> subtree={!r} json={} output_keys={}".format(goal.subtree, goal.json, list(goal.output_keys)))
        send = self.client.send_goal_async(goal)
        if not self.wait(send, 10.0) or not send.result().accepted:
            pytest.fail("goal for subtree '{}' not accepted".format(subtree))
        goal_handle = send.result()
        result_future = goal_handle.get_result_async()

        if cancel_after is not None:
            self.wait(result_future, cancel_after)
            if not result_future.done():
                print("--> canceling after {}s".format(cancel_after))
                self.wait(goal_handle.cancel_goal_async(), 5.0)

        if not self.wait(result_future, timeout):
            self.wait(goal_handle.cancel_goal_async(), 5.0)
            pytest.fail("subtree '{}' gave no result within {}s".format(subtree, timeout))

        state = result_future.result().status
        result = result_future.result().result
        raw = result.json
        print("<-- state={} success={} json={}".format(status_name(state), result.success, raw))

        try:
            parsed = json.loads(raw) if raw else {}
        except ValueError as e:
            pytest.fail("result json is not parseable: {} -- raw was {!r}".format(e, raw))
        return state, parsed, result


@pytest.fixture(scope="session")
def runner(request, ros_node):
    return SubtreeRunner(ros_node, request.config.getoption("--action"), request.config.getoption("--node"))


def gazebo_model_poses():
    """
    Name -> pose of the models in the Gazebo simulation; empty if there is no simulation. Read with the gz tool, in
    the environment the simulation was started in, as ROS doesn't bridge the world's poses.
    """
    def gz(*args):
        try:
            return subprocess.run(["gz"] + list(args), capture_output=True, text=True, timeout=20).stdout
        except (OSError, subprocess.TimeoutExpired):
            return ""

    models = set(re.findall(r"^\s*- (\S+)$", gz("model", "--list"), re.M))
    poses = {}
    # the world's models come first, followed by their links and visuals, that can share names
    for block in re.findall(r"^pose \{\n(.*?)^\}", gz("topic", "-e", "-n", "1", "-t", "/world/default/pose/info"),
                            re.M | re.S):
        name = re.search(r'^  name: "([^"]*)"', block, re.M).group(1)
        if name not in models or name in poses:
            continue

        def field(section, axis):
            match = re.search(r"^  {} \{{[^}}]*?^    {}: (\S+)".format(section, axis), block, re.M | re.S)
            return float(match.group(1)) if match else 0.0

        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = (field("position", a) for a in "xyz")
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = \
            (field("orientation", a) for a in "xyzw")
        poses[name] = pose
    return poses


@pytest.fixture(scope="session")
def initial_model_poses(ros_node):
    """
    The models' poses when the session starts, to put them back before each test, and after the last one. So start
    the first session on a fresh world: the objects where they were spawned.
    """
    return gazebo_model_poses()


@pytest.fixture(scope="session")
def reset_scene(runner, initial_model_poses):
    """
    Put the world back to how it started, as a function to call once the test knows it will use the robot.

    Two parts. The robot's, through bt_server's test_reset_scene tree: the gripper empty, first,
    as an object MoveIt believes attached survives everything else, and then pickups fail on it;
    the arm resting, out of the camera's view; and MoveIt's planning scene empty, for the next
    detect_objects to fill it from what the camera can actually see. Then the world's, on
    simulation: every model back to its pose at the start of the session, so the objects are
    back on the table, and Thorp at its start, what navigation follows only with Gazebo's ground
    truth localization; and with navigation, its costmaps cleared of what they marked before
    the jump. The sleep is for physics: objects put back need a moment to settle
    before a detection of them means anything.

    A function rather than a fixture doing the reset, deliberately: a fixture runs before the
    test body, which means before the body has decided whether it is going to skip. The caller
    resets once it knows it will use the robot. If any did, the world is reset once more at the
    end of the session, to leave it as the session found it, ready for the next one.
    """
    set_pose = runner.node.create_client(SetEntityPose, "/world/default/set_pose")
    used = []

    def reset():
        used.append(True)
        state, out, result = runner.run("test_reset_scene", inputs={}, timeout=60)
        if state != GoalStatus.STATUS_SUCCEEDED or not result.success:
            print("--> resetting the robot failed: {}".format(out or status_name(state)))
        if not initial_model_poses or not set_pose.wait_for_service(timeout_sec=2.0):
            return
        for name, pose in initial_model_poses.items():
            request = SetEntityPose.Request()
            request.entity.name = name
            request.entity.type = Entity.MODEL
            request.pose = pose
            future = set_pose.call_async(request)
            if not runner.wait(future, 5.0) or not future.result().success:
                print("--> putting {} back failed".format(name))
        for costmap in ("local", "global"):
            clear = runner.node.create_client(
                ClearEntireCostmap, "/{0}_costmap/clear_entirely_{0}_costmap".format(costmap))
            if clear.wait_for_service(timeout_sec=1.0):
                runner.wait(clear.call_async(ClearEntireCostmap.Request()), 5.0)
        time.sleep(1.0)

    yield reset
    if used:
        reset()
