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
well, and reading that absence as a broken promise would fail on the good outcome.

Trees needing a stack you haven't declared with --stack are skipped, as are those marked
`status: blocked`, so the suite reports the gaps instead of hiding them:

    cd /catkin_ws/src/thorp/thorp_bt_cpp && python3 -m pytest test/test_capabilities.py -v
    cd /catkin_ws/src/thorp/thorp_bt_cpp && python3 -m pytest test/test_capabilities.py -v --stack navigation
    cd /catkin_ws/src/thorp/thorp_bt_cpp && python3 -m pytest test/test_capabilities.py -v -k pickup_object
"""

import warnings

import pytest
from actionlib_msgs.msg import GoalStatus

from conftest import reset_scene, status_name

POSE_FIELDS = {"x", "y", "z", "roll", "pitch", "yaw", "frame"}

# How many times a setup step may be attempted: MoveIt's planner is sampling-based, so the
# same pick from the same pose fails to plan now and then. Retrying hides nothing, since the
# capability under test still gets a single attempt.
SETUP_ATTEMPTS = 2

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
    # collision objects come back as the names the detector gave them, not as geometry: the
    # geometry lives in the planning scene, and a name is what the manipulation capabilities take
    "moveit_msgs::CollisionObject": str,
    "std::vector<moveit_msgs::CollisionObject>": list,
    # a table comes back as the fields the trees actually read: name, width, depth, height, pose
    "rail_manipulation_msgs::SegmentedObject": dict,
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

    # Plain alphabetical, and the order carries no meaning: clean_scene resets the world
    # before every test, so nothing a test does reaches the next one.
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


def step_label(step):
    """What a step's results are filed under: its `as` name, or the capability's own name."""
    return step.get("as") or step["subtree"]


def setup_steps(spec, capabilities):
    """(name, its spec, the step as written) for each step of a capability's setup."""
    for step in (spec.get("test") or {}).get("setup") or []:
        yield step["subtree"], capabilities.get(step["subtree"], {}), step


def resolve_inputs(block, produced):
    """
    The inputs for one call: the literals in `inputs`, plus whatever `inputs_from` pulls out
    of the results of earlier setup steps.

    `inputs_from` is what lets a chain run on real values instead of numbers someone typed in.
    `{table: [detect_table, table]}` means "the table detect_table found", and a third element
    indexes into a list, so `[table_approach_poses, approach_poses, 0]` is the first of the
    poses computed around it. This is exactly the threading an agent does by hand -- take a
    field out of one result, put it in the next goal -- so a chain that works here is one ROSA
    can follow, and one that can't be expressed here probably can't be asked of ROSA either.

    Raises LookupError naming what was missing, so the caller can report it as a precondition
    that wasn't met rather than as a mysterious type error inside bt_server.
    """
    inputs = dict(block.get("inputs") or {})
    for key, ref in (block.get("inputs_from") or {}).items():
        step_name, out_key = ref[0], ref[1]
        if step_name not in produced:
            raise LookupError("{} has not run yet".format(step_name))
        if out_key not in produced[step_name]:
            raise LookupError("{} did not return {} (it returned {})".format(
                step_name, out_key, sorted(produced[step_name]) or "nothing"))
        value = produced[step_name][out_key]
        if len(ref) > 2:
            index = ref[2]
            if not isinstance(value, list) or len(value) <= index:
                raise LookupError("{}.{} has no element {} (it is {!r})".format(
                    step_name, out_key, index, value))
            value = value[index]
        inputs[key] = value
    return inputs


@pytest.fixture(autouse=True)
def runnable(capability, available_stacks, capabilities, request):
    """
    Whether this capability can be exercised at all, decided before anything touches the robot.

    A fixture rather than the first few lines of the test, so that the world reset below can
    depend on it: a skip raised here stops `clean_scene` from ever running, which is what keeps
    a suite of mostly-skipped tests from resetting gazebo once per skip.
    """
    name, spec = capability

    if spec.get("kind") == "app" and not request.config.getoption("--apps"):
        pytest.skip("app, not a capability: the agent is never offered it, and running one "
                    "here means canceling it and guessing (pass --apps to try anyway)")

    if spec.get("status") == "blocked":
        pytest.skip("blocked: {}".format(" ".join(spec.get("blocked", "no reason given").split())))

    # The setup steps are capabilities too, so whatever they need has to be running as well:
    # pickup_object is manipulation, but the detect_objects that gives it something to pick is
    # perception. Rolling them together means an incomplete --stack skips with a reason rather
    # than failing halfway through the precondition.
    needed = set(spec.get("stack", ["none"]))
    for _, step_spec, _ in setup_steps(spec, capabilities):
        needed |= set(step_spec.get("stack", ["none"]))
    needed -= {"none"}
    missing = needed - available_stacks
    if missing:
        pytest.skip("needs {} running (pass --stack {})".format(
            ",".join(sorted(missing)), ",".join(sorted(needed))))


@pytest.fixture(autouse=True)
def clean_scene(runnable):
    """
    The same starting world for every test that is actually going to run one.

    Depends on `runnable` purely for the ordering: pytest builds fixtures in dependency order,
    so a capability that skips never gets here. Without it the suite is order-dependent in
    effect even though each test establishes its own setup, since pickup_objects clears the
    table and whatever ran after it found nothing to work with.
    """
    reset_scene()


def test_capability(runner, capability, capabilities):
    name, spec = capability

    # Put the robot in the state this capability assumes. These calls go through the same
    # action as the capability under test, which is the point: the precondition is written in
    # the same vocabulary the agent will use to reach it, not staged behind the robot's back.
    #
    # A step that fails SKIPS rather than fails. "pickup_object is broken" and "nothing was
    # detected for it to pick up" are different findings, and a red test that means the second
    # one teaches you to ignore red tests.
    produced = {}
    for step_name, step_spec, step in setup_steps(spec, capabilities):
        try:
            step_inputs = resolve_inputs(step, produced)
        except LookupError as e:
            pytest.skip("precondition not established: {} needs {}".format(step_name, e))
        for attempt in range(1, SETUP_ATTEMPTS + 1):
            # every documented output, since a later step may want any of them
            step_state, step_out, step_result = runner.run(
                step_name, inputs=step_inputs,
                output_keys=sorted(step_spec.get("outputs") or {}),
                timeout=(step_spec.get("test") or {}).get("timeout", 120))
            if step_state == GoalStatus.SUCCEEDED and step_result.success:
                break
            print("--> setup step {} failed on attempt {} of {}: {}".format(
                step_name, attempt, SETUP_ATTEMPTS, step_out or "no detail"))
        else:
            # Which of the two failed: the action completing and the tree succeeding are
            # different things, and the action state alone reads as a success either way.
            outcome = ("the tree returned FAILURE" if step_state == GoalStatus.SUCCEEDED
                       else "the goal ended as {}".format(status_name(step_state)))
            pytest.skip("precondition not established in {} attempts: {} {}{}".format(
                SETUP_ATTEMPTS, step_name, outcome,
                "; {}".format(step_out) if step_out else ""))
        produced[step_label(step)] = step_out

    test = spec.get("test", {})
    declared = spec.get("outputs") or {}
    # Ask for everything the capability documents. What comes back and what *has* to come back
    # are different questions, settled further down by each output's `when` -- narrowing the
    # request instead would just hide the outputs that are hard to predict, which are the ones
    # worth watching. A test block can still override this where a key is genuinely unwanted.
    output_keys = test.get("output_keys", sorted(declared))
    try:
        inputs = resolve_inputs(test, produced)
    except LookupError as e:
        pytest.skip("precondition not established: this capability needs {}".format(e))
    state, out, result = runner.run(name,
                                    inputs=inputs,
                                    output_keys=output_keys,
                                    timeout=test.get("timeout", 60),
                                    cancel_after=test.get("cancel_after"))

    # the contract: the server accepted and served the goal. An aborted goal means bt_server
    # refused it -- unknown tree, bad json, missing input -- which is our problem, not the
    # robot's, so it fails regardless of `expect`.
    assert state != GoalStatus.ABORTED, \
        "goal was refused: {}".format(out.get("error", out))
    assert state in (GoalStatus.SUCCEEDED, GoalStatus.PREEMPTED), status_name(state)

    # Outputs a real run has to produce, asserted whatever `expect` says. For a tree whose
    # status carries no information this is the only thing that separates a run that did the
    # job from one that didn't: hunt_cat reports SUCCESS through a ForceSuccess having seen no
    # cat at all, and only cannon_tilt_angle, written by AimCannon, tells the two apart.
    absent = [key for key in test.get("expect_outputs") or [] if key not in out]
    assert not absent, "{} produced none of {}; outputs were {}".format(
        name, absent, out or "none")

    expect = test.get("expect", "any")
    if expect == "succeeded":
        assert result.success, "tree reported FAILURE; outputs were {}".format(out)
    elif expect == "failed":
        assert not result.success, "tree unexpectedly succeeded"
    elif expect == "runs":
        # An app-level tree that runs until told to stop -- KeepRunningUntilFailure, or a
        # Repeat with no cycle count. "Did it finish" is meaningless for one of those; what
        # can be checked is that it accepted the goal, got far enough to put something on the
        # blackboard, and stopped cleanly when canceled.
        assert state == GoalStatus.PREEMPTED, \
            "expected to still be running at the cancel, but ended as {}".format(status_name(state))
        assert out, "ran for {}s without producing any of {}".format(test.get("cancel_after"), output_keys)
    elif not result.success:
        # `expect: any` means the outcome isn't asserted, not that nobody should hear about
        # it: the test passes, and a tree failing every run would otherwise leave nothing red
        # anywhere. A warning lands in pytest's summary -- visible, not fatal.
        warnings.warn("{} returned FAILURE; not asserted because expect: any. outputs: {}".format(
            name, out or "none"), UserWarning)
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
    # that may never fire, or by a node a successful run can walk straight past -- the untaken
    # half of a Fallback, a switch case, an empty loop. Neither is asserted in either direction:
    # a fallback that recovered from an aborted action legitimately leaves an error code behind
    # on an otherwise successful run. A run that was canceled, or failed partway, is exempt
    # from all of this.
    if result.success:
        guaranteed = {k for k in output_keys
                      if k in declared and declared[k].get("when", "success") == "success"}
        unserializable = {k for k in guaranteed
                          if declared[k].get("type") not in SERIALIZED_AS}
        missing_outputs = guaranteed - set(out) - unserializable
        assert not missing_outputs, \
            "capability claims {} on success but the run didn't produce them".format(
                sorted(missing_outputs))
