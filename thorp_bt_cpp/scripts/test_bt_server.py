#!/usr/bin/env python3
"""
End-to-end check of thorp_bt_cpp's bt_server RunSubtree action, against the trees in
bt/test_server.xml. Needs nothing but a roscore and bt_server itself: no robot, no
simulator, no other node.

    roscore &
    rosrun thorp_bt_cpp bt_server_node _bt_dir:=$(rospack find thorp_bt_cpp)/bt
    rosrun thorp_bt_cpp test_bt_server.py

Reports each case as PASS/FAIL and exits non-zero if anything failed, so it can be dropped
into CI later.
"""

import argparse
import json
import sys

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus

from thorp_msgs.msg import RunSubtreeAction, RunSubtreeGoal

# what the trees read from bt_server's private namespace, and what we expect back out
PARAMS = {
    "test_float": 1.5,
    "test_int": 42,
    "test_string": "hello",
    "test_bool": True,
}

START_POSE = "1.0;2.0;0.0;map"  # x;y;yaw;frame, the same form a literal xml attribute uses
OFFSET_X = 0.5

POSE_FIELDS = {"x", "y", "z", "roll", "pitch", "yaw", "frame"}

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


class Report:
    """Collects check results so one failure doesn't stop the rest of the run."""

    def __init__(self):
        self.failures = 0
        self.checks = 0

    def check(self, ok, description, detail=""):
        self.checks += 1
        if not ok:
            self.failures += 1
        print("  [{}] {}{}".format("PASS" if ok else "FAIL", description,
                                   "" if not detail else "  --> {}".format(detail)))
        return ok

    def summary(self):
        print("\n{} check(s), {} failure(s)".format(self.checks, self.failures))
        return 0 if self.failures == 0 else 1


class Client:
    def __init__(self, action_name, timeout):
        self.timeout = timeout
        self.client = actionlib.SimpleActionClient(action_name, RunSubtreeAction)
        print("Waiting for action server '{}'...".format(action_name))
        if not self.client.wait_for_server(rospy.Duration(10.0)):
            sys.exit("Action server '{}' not available -- is bt_server_node running?".format(action_name))

    def run(self, subtree, inputs=None, output_keys=None, raw_json=None, cancel_after=None):
        """Sends one goal and returns (goal_state, parsed_result_json, raw_result).

        `raw_json` bypasses json.dumps, for deliberately malformed payloads.
        """
        goal = RunSubtreeGoal()
        goal.subtree = subtree
        goal.json = raw_json if raw_json is not None else (json.dumps(inputs) if inputs is not None else "")
        goal.output_keys = output_keys or []

        print("\n  --> subtree={!r} json={} output_keys={}".format(goal.subtree, goal.json, list(goal.output_keys)))
        self.client.send_goal(goal)

        if cancel_after is not None:
            rospy.sleep(cancel_after)
            print("  --> canceling after {}s".format(cancel_after))
            self.client.cancel_goal()

        if not self.client.wait_for_result(rospy.Duration(self.timeout)):
            print("  <-- TIMED OUT after {}s".format(self.timeout))
            return None, {}, None

        state = self.client.get_state()
        result = self.client.get_result()
        raw = result.json if result else ""
        print("  <-- state={} success={} json={}".format(
            status_name(state), result.success if result else "?", raw))

        try:
            parsed = json.loads(raw) if raw else {}
        except ValueError as e:
            print("  <-- result json is not parseable: {}".format(e))
            parsed = {}
        return state, parsed, result


def case_round_trip(client, report):
    """Inputs seeded from json, named outputs read back -- the path ROSA will use."""
    print("\n=== 1. round trip with explicit output_keys ===")
    wanted = ["moved_pose", "a_float", "an_int", "a_string", "a_bool"]
    state, out, result = client.run(
        "test_server",
        inputs={"start_pose": START_POSE, "offset_x": OFFSET_X},
        output_keys=wanted,
    )

    report.check(state == GoalStatus.SUCCEEDED, "goal succeeded", status_name(state))
    report.check(result is not None and result.success, "tree reported SUCCESS")
    report.check(set(out.keys()) == set(wanted), "result holds exactly the requested keys",
                 "got {}".format(sorted(out.keys())))

    # typed values survive as their json counterparts, not as the strings we seeded
    report.check(isinstance(out.get("a_float"), float) and abs(out["a_float"] - PARAMS["test_float"]) < 1e-5,
                 "float output is a json number", repr(out.get("a_float")))
    report.check(out.get("an_int") == PARAMS["test_int"] and isinstance(out.get("an_int"), int),
                 "int output is a json integer", repr(out.get("an_int")))
    report.check(out.get("a_string") == PARAMS["test_string"], "string output round-trips",
                 repr(out.get("a_string")))
    report.check(out.get("a_bool") is True, "bool output is a json bool", repr(out.get("a_bool")))

    # the pose came back nested, with every field, rather than as a ';' separated string
    pose = out.get("moved_pose")
    if report.check(isinstance(pose, dict), "pose output is a nested object", repr(pose)):
        report.check(set(pose.keys()) == POSE_FIELDS, "pose has the full fixed field set",
                     "got {}".format(sorted(pose.keys())))
        report.check(pose.get("frame") == "map", "pose kept its frame", repr(pose.get("frame")))
        # proves the seeded pose really was parsed out of the input json and used
        expected_x = 1.0 + OFFSET_X
        report.check(abs(pose.get("x", 0.0) - expected_x) < 1e-3,
                     "seeded pose was parsed and transformed (x == {})".format(expected_x),
                     "x={} y={}".format(pose.get("x"), pose.get("y")))


def case_unsupported_type(client, report):
    """A blackboard value we have no serializer for must be visible, not silently missing."""
    print("\n=== 2. output key holding an unserializable type ===")
    state, out, _ = client.run(
        "test_server",
        inputs={"start_pose": START_POSE, "offset_x": OFFSET_X},
        output_keys=["a_path"],
    )
    report.check(state == GoalStatus.SUCCEEDED, "goal succeeded", status_name(state))
    value = out.get("a_path")
    report.check(isinstance(value, str) and value.startswith("<unsupported type:"),
                 "nav_msgs::Path reported as an unsupported-type tag", repr(value))


def case_missing_key(client, report):
    """A key that never gets a value is omitted, so the caller can diff against its request."""
    print("\n=== 3. output key that the run never sets ===")
    state, out, result = client.run(
        "test_server",
        inputs={"start_pose": START_POSE, "offset_x": OFFSET_X},
        output_keys=["a_string", "no_such_key"],
    )
    report.check(state == GoalStatus.SUCCEEDED, "goal succeeded", status_name(state))
    report.check(result is not None and result.success, "tree still reported SUCCESS")
    report.check("no_such_key" not in out, "absent key omitted from the result")
    report.check("a_string" in out, "the other requested key still came back")


def case_discovery_fallback(client, report):
    """Empty output_keys: report what the run added, without the inputs we seeded."""
    print("\n=== 4. discovery mode (no output_keys) ===")
    state, out, _ = client.run(
        "test_server",
        inputs={"start_pose": START_POSE, "offset_x": OFFSET_X},
        output_keys=[],
    )
    report.check(state == GoalStatus.SUCCEEDED, "goal succeeded", status_name(state))
    produced = {"moved_pose", "a_float", "an_int", "a_string", "a_bool", "a_path"}
    report.check(produced.issubset(set(out.keys())), "every key the run produced is reported",
                 "missing {}".format(sorted(produced - set(out.keys()))))
    report.check("start_pose" not in out and "offset_x" not in out,
                 "seeded inputs are not echoed back", "got {}".format(sorted(out.keys())))


def case_unknown_subtree(client, report):
    """A bad request aborts, rather than coming back as a failed-but-served goal."""
    print("\n=== 5. unknown subtree ===")
    state, out, result = client.run("no_such_tree", inputs={})
    report.check(state == GoalStatus.ABORTED, "goal aborted", status_name(state))
    report.check(result is not None and not result.success, "success is false")
    report.check("error" in out, "result carries an error message", repr(out))


def case_malformed_json(client, report):
    """Bad input json is a request-level error, so it aborts rather than running anything."""
    print("\n=== 6. malformed input json ===")
    state, out, result = client.run("test_server", raw_json="{not valid json")
    report.check(state == GoalStatus.ABORTED, "goal aborted", status_name(state))
    report.check(result is not None and not result.success, "success is false")
    report.check("error" in out, "result carries an error message", repr(out))


def case_preemption(client, report):
    """Canceling mid-run returns whatever outputs existed at that point."""
    print("\n=== 7. preemption returns partial outputs ===")
    state, out, result = client.run(
        "test_server_slow",
        inputs={"start_pose": START_POSE, "offset_x": OFFSET_X},
        output_keys=["moved_pose", "late_value"],
        cancel_after=1.0,
    )
    report.check(state == GoalStatus.PREEMPTED, "goal preempted", status_name(state))
    report.check(result is not None and not result.success, "success is false")
    report.check("moved_pose" in out, "output set before the cancel is still reported")
    report.check("late_value" not in out, "output the run never reached is omitted")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--action", default="/run_subtree", help="RunSubtree action name")
    parser.add_argument("--param-ns", default="/bt_server",
                        help="bt_server's private namespace, where the trees read parameters from")
    parser.add_argument("--timeout", type=float, default=30.0, help="per goal timeout, seconds")
    args, _ = parser.parse_known_args(rospy.myargv()[1:])

    rospy.init_node("test_bt_server", anonymous=True)

    for name, value in PARAMS.items():
        rospy.set_param("{}/{}".format(args.param_ns.rstrip("/"), name), value)
    print("Set test parameters under {}".format(args.param_ns))

    client = Client(args.action, args.timeout)
    report = Report()

    case_round_trip(client, report)
    case_unsupported_type(client, report)
    case_missing_key(client, report)
    case_discovery_fallback(client, report)
    case_unknown_subtree(client, report)
    case_malformed_json(client, report)
    case_preemption(client, report)

    sys.exit(report.summary())


if __name__ == "__main__":
    main()
