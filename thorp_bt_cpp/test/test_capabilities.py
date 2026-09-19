"""
Every tree in config/capabilities.yaml, driven through RunSubtree.

One test per capability, parametrized from the yaml rather than written out by hand: each one
does the same three things (send the goal, wait, check the result) and only the data differs,
so a script per tree would be the same logic copied a dozen times, drifting.

What gets asserted is the *contract*, not the robot: that the goal was served rather than
refused, that the outputs the capability promises came back, and that they have the shape the
yaml claims. Whether a real pickup actually succeeds depends on where the objects happen to
be, so `expect: any` means the physical outcome is reported and not asserted -- a flaky
assertion nobody trusts is worse than none. Capabilities declaring `expect: succeeded` are
held to it.

Asking for an output and insisting on one are kept apart, because they answer to different
things. Every documented output is requested; only those the yaml marks `when: success` are
insisted on. An error code written in an action's onAborted is absent from a run that went
well, and a test that read that absence as a broken promise would be failing on the good
outcome -- which is exactly what it did before `when` existed.

Trees needing a stack you haven't declared with --stack are skipped, as are those marked
`status: blocked`, so the suite reports the gaps instead of hiding them:

    cd /catkin_ws/src/thorp/thorp_bt_cpp && python3 -m pytest test/test_capabilities.py -v
    cd /catkin_ws/src/thorp/thorp_bt_cpp && python3 -m pytest test/test_capabilities.py -v --stack navigation
    cd /catkin_ws/src/thorp/thorp_bt_cpp && python3 -m pytest test/test_capabilities.py -v -k pickup_object
"""

import pytest
from actionlib_msgs.msg import GoalStatus

from conftest import status_name

POSE_FIELDS = {"x", "y", "z", "roll", "pitch", "yaw", "frame"}

# what each declared type should look like once it has been through bt_server's serializer.
# Anything not listed is a type we can't serialize, and comes back as an "<unsupported ...>"
# tag -- checked for separately.
SERIALIZED_AS = {
    "string": str,
    "std::string": str,
    "bool": bool,
    "int": int,
    "unsigned int": int,
    "unsigned long": int,
    "float": float,
    "double": float,
    "geometry_msgs::PoseStamped": dict,
    "std::vector<geometry_msgs::PoseStamped>": list,
}


def pytest_generate_tests(metafunc):
    """One test per capability, named after it so -k works on tree names."""
    if "capability" not in metafunc.fixturenames:
        return
    import os
    import yaml

    path = metafunc.config.getoption("--capabilities")
    if not path:
        here = os.path.dirname(os.path.abspath(__file__))
        path = os.path.join(here, os.pardir, "config", "capabilities.yaml")
    with open(path) as f:
        caps = yaml.safe_load(f)["capabilities"]

    metafunc.parametrize("capability", sorted(caps.items()), ids=lambda item: item[0])


def check_value(name, value, declared_type):
    """A returned output either matches its declared type, or says why it couldn't."""
    if isinstance(value, str) and value.startswith("<unsupported type:"):
        # fine for a type we never claimed to handle; a bug if the yaml says otherwise
        assert declared_type not in SERIALIZED_AS, (
            "'{}' is declared as {}, which bt_server should be able to serialize, "
            "but it came back as {}".format(name, declared_type, value))
        return

    expected = SERIALIZED_AS.get(declared_type)
    if expected is None:
        return  # type we never claimed to serialize, and it wasn't tagged; nothing to check
    if expected is float:
        assert isinstance(value, (int, float)), "{}={!r} should be a number".format(name, value)
    else:
        assert isinstance(value, expected), \
            "{}={!r} should be {} for declared type {}".format(name, value, expected.__name__, declared_type)

    if declared_type == "geometry_msgs::PoseStamped":
        assert set(value) == POSE_FIELDS, "pose {} has fields {}".format(name, sorted(value))


def test_capability(runner, capability, available_stacks):
    name, spec = capability

    if spec.get("status") == "blocked":
        pytest.skip("blocked: {}".format(" ".join(spec.get("blocked", "no reason given").split())))

    needed = set(spec.get("stack", ["none"])) - {"none"}
    missing = needed - available_stacks
    if missing:
        pytest.skip("needs {} running (pass --stack {})".format(
            ",".join(sorted(missing)), ",".join(sorted(needed))))

    test = spec.get("test", {})
    declared = spec.get("outputs") or {}
    # Ask for everything the capability documents. What comes back and what *has* to come back
    # are different questions, settled further down by each output's `when` -- narrowing the
    # request instead would just hide the outputs that are hard to predict, which are the ones
    # worth watching. A test block can still override this where a key is genuinely unwanted.
    output_keys = test.get("output_keys", sorted(declared))
    state, out, result = runner.run(name,
                                    inputs=test.get("inputs", {}),
                                    output_keys=output_keys,
                                    timeout=test.get("timeout", 60),
                                    cancel_after=test.get("cancel_after"))

    # the contract: the server accepted and served the goal. An aborted goal means bt_server
    # refused it -- unknown tree, bad json, missing input -- which is our problem, not the
    # robot's, so it fails regardless of `expect`.
    assert state != GoalStatus.ABORTED, \
        "goal was refused: {}".format(out.get("error", out))
    assert state in (GoalStatus.SUCCEEDED, GoalStatus.PREEMPTED), status_name(state)

    expect = test.get("expect", "any")
    if expect == "succeeded":
        assert result.success, "tree reported FAILURE; outputs were {}".format(out)
    elif expect == "failed":
        assert not result.success, "tree unexpectedly succeeded"
    elif expect == "runs":
        # An app-level tree that runs until told to stop -- KeepRunningUntilFailure, or a
        # Repeat with no cycle count. "Did it finish" is meaningless for one of those; what
        # can be checked is that it accepted the goal, got far enough to put something on the
        # blackboard, and stopped cleanly when cancelled.
        assert state == GoalStatus.PREEMPTED, \
            "expected to still be running at the cancel, but ended as {}".format(status_name(state))
        assert out, "ran for {}s without producing any of {}".format(test.get("cancel_after"), output_keys)
    else:
        print("outcome not asserted (expect: any): success={}".format(result.success))

    # whatever did come back has to match what the capability promises
    for key, value in out.items():
        if key in declared:
            check_value(key, value, declared[key].get("type", ""))

    # A tree that ran to completion should have produced what it promises on the path that
    # succeeds -- and only that. An output marked `when: failure` is an error report written in
    # an action's onAborted, so a run that succeeded is *right* not to have one, and demanding
    # it would make the good outcome the failing case. `when: maybe` is written from a callback
    # that may never fire. Neither is asserted in either direction: a fallback that recovered
    # from an aborted action legitimately leaves an error code behind on an otherwise
    # successful run. A run that was cancelled, or failed partway, is exempt from all of this.
    if result.success:
        guaranteed = {k for k in output_keys
                      if k in declared and declared[k].get("when", "success") == "success"}
        unserializable = {k for k in guaranteed
                          if declared[k].get("type") not in SERIALIZED_AS}
        missing_outputs = guaranteed - set(out) - unserializable
        assert not missing_outputs, \
            "capability claims {} on success but the run didn't produce them".format(
                sorted(missing_outputs))
