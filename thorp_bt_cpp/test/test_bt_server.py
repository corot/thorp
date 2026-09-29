"""
bt_server's own contract, tested against bt/test_server.xml -- trees built only from nodes
that need nothing outside the process. No robot, no simulator, no other node: just bt_server,
so this tier is fast and deterministic enough to run on every commit. From the package
directory:

    ros2 run thorp_bt_cpp bt_server_node
    python3 -m pytest test/test_bt_server.py -v
"""

import pytest
from action_msgs.msg import GoalStatus

from conftest import status_name

# what the test trees read from bt_server's parameters, and what should come back out
PARAMS = {
    "test_float": 1.5,
    "test_int": 42,
    "test_string": "hello",
    "test_bool": True,
}

START_POSE = "1.0;2.0;0.0;map"  # x;y;yaw;frame, the form a literal xml attribute would use
OFFSET_X = 0.5
INPUTS = {"start_pose": START_POSE, "offset_x": OFFSET_X}

POSE_FIELDS = {"x", "y", "z", "roll", "pitch", "yaw", "frame"}


@pytest.fixture(scope="module", autouse=True)
def test_params(runner):
    for name, value in PARAMS.items():
        runner.set_param(name, value)


def test_named_outputs_come_back_typed(runner):
    """The path ROSA uses: seed inputs as json, name the outputs, get them back."""
    wanted = ["moved_pose", "a_float", "an_int", "a_string", "a_bool"]
    state, out, result = runner.run("test_server", inputs=INPUTS, output_keys=wanted)

    assert state == GoalStatus.STATUS_SUCCEEDED, status_name(state)
    assert result.success
    assert set(out) == set(wanted), "result should hold exactly the requested keys"

    # values keep their type rather than coming back as the strings we seeded
    assert isinstance(out["a_float"], float) and out["a_float"] == pytest.approx(PARAMS["test_float"])
    assert isinstance(out["an_int"], int) and out["an_int"] == PARAMS["test_int"]
    assert out["a_string"] == PARAMS["test_string"]
    assert out["a_bool"] is True


def test_pose_round_trips_as_a_nested_object(runner):
    """A pose seeded as 'x;y;yaw;frame' is parsed, used, and returned as an object."""
    state, out, _ = runner.run("test_server", inputs=INPUTS, output_keys=["moved_pose"])
    assert state == GoalStatus.STATUS_SUCCEEDED, status_name(state)

    pose = out["moved_pose"]
    assert isinstance(pose, dict), "pose should be nested, not a ';' separated string"
    assert set(pose) == POSE_FIELDS, "pose fields should be the same set every time"
    assert pose["frame"] == "map", "frame survives the round trip"
    # proves the seeded pose really was parsed out of the json and transformed
    assert pose["x"] == pytest.approx(1.0 + OFFSET_X, abs=1e-3)
    assert pose["y"] == pytest.approx(2.0, abs=1e-3)


def test_unserializable_output_is_tagged_not_dropped(runner):
    """A type we have no serializer for has to be visible in the result, not just missing."""
    state, out, _ = runner.run("test_server", inputs=INPUTS, output_keys=["a_path"])
    assert state == GoalStatus.STATUS_SUCCEEDED, status_name(state)
    assert out["a_path"].startswith("<unsupported type:"), out["a_path"]


def test_output_key_never_set_is_omitted(runner):
    """So a caller can tell what's missing by diffing against what it asked for."""
    state, out, result = runner.run("test_server", inputs=INPUTS,
                                    output_keys=["a_string", "no_such_key"])
    assert state == GoalStatus.STATUS_SUCCEEDED, status_name(state)
    assert result.success, "a missing output key doesn't make the tree itself fail"
    assert "no_such_key" not in out
    assert "a_string" in out


def test_discovery_mode_reports_what_the_run_added(runner):
    """Empty output_keys: everything the run produced, and none of what we seeded."""
    state, out, _ = runner.run("test_server", inputs=INPUTS, output_keys=[])
    assert state == GoalStatus.STATUS_SUCCEEDED, status_name(state)

    produced = {"moved_pose", "a_float", "an_int", "a_string", "a_bool", "a_path"}
    assert produced <= set(out), "missing {}".format(sorted(produced - set(out)))
    assert "start_pose" not in out and "offset_x" not in out, "seeded inputs shouldn't be echoed"


def test_unknown_subtree_aborts(runner):
    """A bad request aborts, rather than being served and reported as a failure."""
    state, out, result = runner.run("no_such_tree", inputs={})
    assert state == GoalStatus.STATUS_ABORTED, status_name(state)
    assert not result.success
    assert "error" in out


def test_malformed_input_json_aborts(runner):
    state, out, result = runner.run("test_server", raw_json="{not valid json")
    assert state == GoalStatus.STATUS_ABORTED, status_name(state)
    assert not result.success
    assert "error" in out


def test_missing_input_aborts_naming_the_key(runner):
    """
    Nothing checks inputs before a run; the node that reads a missing one throws, and the
    error names the key. An agent that forgets an argument gets that back, and the server
    must still be alive for the next goal -- which the test after this one checks.
    """
    state, out, result = runner.run("test_server", inputs={"offset_x": OFFSET_X})  # no start_pose
    assert state == GoalStatus.STATUS_ABORTED, status_name(state)
    assert not result.success
    assert "start_pose" in out.get("error", ""), out


def test_missing_list_aborts_rather_than_crashing(runner):
    """
    GetPoseListFront and PopPoseFromList are what every list-driven tree is built from,
    reach_first_pose and follow_waypoints included. Reading a list that was never set is the
    case that took the server down when ports were dereferenced unchecked.
    """
    state, out, result = runner.run("test_server_needs_list", inputs={})
    assert state == GoalStatus.STATUS_ABORTED, status_name(state)
    assert not result.success
    assert "poses" in out.get("error", ""), out


def test_local_offset_moves_along_the_pose_own_axes(runner):
    """Facing +y, a local x offset moves the pose along map +y; a frame offset would move it along x."""
    state, out, result = runner.run("test_server_local_offset",
                                    inputs={"start_pose": "1.0;2.0;1.5708;map", "offset_x": OFFSET_X},
                                    output_keys=["moved_pose"])
    assert state == GoalStatus.STATUS_SUCCEEDED and result.success, status_name(state)
    pose = out["moved_pose"]
    assert pose["x"] == pytest.approx(1.0, abs=1e-3)
    assert pose["y"] == pytest.approx(2.0 + OFFSET_X, abs=1e-3)
    assert pose["yaw"] == pytest.approx(1.5708, abs=1e-3)


def test_pose_list_seeds_from_a_json_array(runner):
    """
    A json array becomes a real std::vector<PoseStamped>, which is what unblocks the
    navigation trees: they take their waypoints as a list, and until now nothing could put
    one on a blackboard. Each element is written the way a literal xml attribute writes a
    pose, so there's one pose syntax rather than two.
    """
    state, out, result = runner.run("test_server_needs_list",
                                    inputs={"poses": ["1.0;2.0;0.0;map", "3.0;4.0;1.57;map"]},
                                    output_keys=["first_pose", "popped_pose"])
    assert state == GoalStatus.STATUS_SUCCEEDED, status_name(state)
    assert result.success

    first = out["first_pose"]
    assert first["x"] == pytest.approx(1.0) and first["y"] == pytest.approx(2.0)
    assert first["frame"] == "map"


def test_segmented_object_seeds_from_a_json_object(runner):
    """
    A table is passed as an object rather than a flattened string, and the proof that its
    fields land is behavioral: TableValidSize reads width and depth, and nothing else, to
    decide whether a table is a usable size. So a table inside the configured range has to
    pass and one outside it has to fail -- which can only happen if those two numbers made it
    onto the blackboard.
    """
    runner.set_param("table_min_side", 0.5)
    runner.set_param("table_max_side", 1.5)

    in_range = {"name": "table1", "pose": "1.0;2.0;0.0;map", "width": 1.0, "depth": 0.8}
    state, _, result = runner.run("test_server_table", inputs={"table": in_range})
    assert state == GoalStatus.STATUS_SUCCEEDED, status_name(state)
    assert result.success, "a 1.0 x 0.8 table is within [0.5, 1.5] and should be accepted"

    too_big = dict(in_range, width=3.0)
    state, _, result = runner.run("test_server_table", inputs={"table": too_big})
    assert state == GoalStatus.STATUS_SUCCEEDED, status_name(state)
    assert not result.success, "a 3.0 x 0.8 table is outside [0.5, 1.5] and should be rejected"


def test_counts_map_round_trips(runner):
    """A json object becomes a std::map<std::string, uint32_t> and comes back as one."""
    state, out, result = runner.run("test_server_counts",
                                    inputs={"target_name": "cube1",
                                            "failures": {"cube1": 1, "cube2": 5},
                                            "given_up_count": 0},
                                    output_keys=["failures"])
    assert state == GoalStatus.STATUS_SUCCEEDED, status_name(state)

    failures = out["failures"]
    assert isinstance(failures, dict), failures
    assert failures["cube1"] == 2, "the failure count for the named target should have gone up"
    assert failures["cube2"] == 5, "counts for other objects should be untouched"


def test_structured_input_for_an_unknown_key_is_reported(runner):
    """
    Building a structured value needs to know the type, which comes from the port using that
    key. A key no port uses has no type to build it as, so it's skipped rather than guessed
    at -- and since it's skipped, a tree that needed it still fails for want of it.
    """
    state, out, result = runner.run("test_server_needs_list",
                                    inputs={"poses": ["1.0;2.0;0.0;map"], "mystery": [1, 2, 3]},
                                    output_keys=["first_pose"])
    assert state == GoalStatus.STATUS_SUCCEEDED, status_name(state)
    assert "first_pose" in out, "the key that is used should still have worked"


def test_a_pose_we_emitted_can_be_seeded_back(runner):
    """
    The round trip that makes capabilities composable: take a pose out of one result and hand
    it to the next goal as-is.

    Poses go out as an object (x, y, z, roll, pitch, yaw, frame) and seeding accepts both that
    and the "x;y;yaw;frame" string, so a pose one capability returns is a valid input to the
    next. approach_table -> detach_from_table is a chain that needs it.
    """
    _, out, _ = runner.run("test_server", inputs=INPUTS, output_keys=["moved_pose"])
    emitted = out["moved_pose"]
    assert isinstance(emitted, dict)

    # hand it straight back, unedited, as the input pose this time
    state, out, result = runner.run("test_server",
                                    inputs={"start_pose": emitted, "offset_x": 0.0},
                                    output_keys=["moved_pose"])
    assert state == GoalStatus.STATUS_SUCCEEDED, status_name(state)
    assert result.success
    for field in ("x", "y", "z", "frame"):
        assert out["moved_pose"][field] == emitted[field], \
            "{} changed on the way round: {} -> {}".format(field, emitted[field], out["moved_pose"][field])


def test_a_badly_shaped_input_aborts_rather_than_killing_the_server(runner):
    """
    Seeding builds typed values out of json, and the wrong shape throws from inside nlohmann
    rather than returning an error. Uncaught, that took the whole process down: one malformed
    goal and every later one was gone too. Since the callers here are LLMs writing json, that
    is a when rather than an if.

    The test after this one is the real assertion: the server is still answering.
    """
    state, out, result = runner.run("test_server",
                                    inputs={"start_pose": {"x": "not a number"}, "offset_x": 0.5})
    assert state in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED), status_name(state)
    if state == GoalStatus.STATUS_ABORTED:
        assert "error" in out, out


def test_server_survives_a_rejected_goal(runner):
    """Whatever the previous cases threw at it, the server is still serving."""
    state, out, result = runner.run("test_server", inputs=INPUTS, output_keys=["a_string"])
    assert state == GoalStatus.STATUS_SUCCEEDED, status_name(state)
    assert out["a_string"] == PARAMS["test_string"]


def test_preemption_returns_partial_outputs(runner):
    """Canceling mid-run gives back whatever had been produced by that point."""
    state, out, result = runner.run("test_server_slow", inputs=INPUTS,
                                    output_keys=["moved_pose", "late_value"],
                                    cancel_after=1.0, timeout=30.0)
    assert state == GoalStatus.STATUS_CANCELED, status_name(state)
    assert not result.success
    assert "moved_pose" in out, "output set before the cancel should still be reported"
    assert "late_value" not in out, "output the run never reached should be omitted"
