"""
What the agent is offered, checked without ROS, langchain or an API key.

The drift test in thorp_bt_cpp already holds capabilities.yaml to the trees. What is left to
check is the step after: that every capability survives the trip into a tool, that no app
leaks through, and that nothing is described to a model in a way it cannot act on -- an
unnamed argument, a type with no mapping, a description that says nothing.

    cd /catkin_ws/src/thorp/thorp_agent && python3 -m pytest test/ -v
"""

import os

import pytest

import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from thorp_agent import capabilities  # noqa: E402

YAML = os.path.join(os.path.dirname(__file__),
                    "..", "..", "thorp_bt_cpp", "config", "capabilities.yaml")


@pytest.fixture(scope="module")
def document():
    if not os.path.exists(YAML):
        pytest.skip("thorp_bt_cpp/config/capabilities.yaml not found next to this package")
    return capabilities.load(YAML)


@pytest.fixture(scope="module")
def specs(document):
    return capabilities.specs_from(document)


def test_apps_are_not_offered(document):
    """
    The rule that keeps an agent from starting something it cannot stop. cat_hunter and
    explore_house run until canceled; a model that calls one has no way back.
    """
    offered = capabilities.offered(document)
    apps = [name for name, spec in (document.get("capabilities") or {}).items()
            if spec.get("kind") == "app"]
    assert apps, "no apps in the yaml at all -- has kind: app gone away?"
    assert not set(apps) & set(offered)


def test_blocked_capabilities_are_not_offered(document):
    for name, spec in (document.get("capabilities") or {}).items():
        if spec.get("status") == "blocked":
            assert name not in capabilities.offered(document)


def test_every_offered_capability_becomes_a_tool(document, specs):
    assert {spec["name"] for spec in specs} == set(capabilities.offered(document))


def test_arguments_match_the_declared_inputs(document, specs):
    offered = capabilities.offered(document)
    for spec in specs:
        assert set(spec["args"]) == set(offered[spec["name"]].get("inputs") or {}), spec["name"]


def test_every_argument_type_is_one_we_can_render(specs):
    """
    An unmapped type silently becomes a string, and the model then sends a string where a
    pose or a list was wanted. Better to fail here, where the fix is one line in TYPES.
    """
    unmapped = [(spec["name"], field, meta["type"])
                for spec in specs for field, meta in spec["args"].items()
                if meta["type"] not in capabilities.TYPES]
    assert not unmapped, "no rendering for: {}".format(unmapped)


def test_descriptions_say_something(specs):
    for spec in specs:
        assert len(spec["description"]) > 40, spec["name"]
        for field, meta in spec["args"].items():
            assert meta["description"].strip(), "{}.{}".format(spec["name"], field)


def test_outputs_are_advertised_so_values_can_be_threaded(document, specs):
    """
    Each call starts from an empty blackboard, so an agent chains calls only by passing values
    back. It can only do that for outputs it was told about.
    """
    offered = capabilities.offered(document)
    for spec in specs:
        assert spec["outputs"] == sorted(offered[spec["name"]].get("outputs") or {})


def test_every_kind_is_one_capabilities_knows(specs):
    for spec in specs:
        for field, meta in spec["args"].items():
            kind, _ = capabilities.TYPES[meta["type"]]
            assert kind in capabilities.KINDS, "{}.{} is {}".format(spec["name"], field, kind)


def test_every_kind_has_a_python_type():
    """
    A kind with no rendering falls back to a string, and the model then sends a string where
    an object or a list was wanted. Needs langchain, so it skips where that isn't installed.
    """
    pytest.importorskip("langchain_core")
    from thorp_agent import tools
    assert capabilities.KINDS <= set(tools.PY_TYPES)


def test_a_capability_taking_a_pose_says_how_to_write_one(specs):
    posed = [spec for spec in specs
             if any(meta["type"] == capabilities.POSE for meta in spec["args"].values())]
    assert posed, "no capability takes a pose any more?"
    for spec in posed:
        for field, meta in spec["args"].items():
            if meta["type"] == capabilities.POSE:
                assert "x;y;yaw;frame" in meta["description"], "{}.{}".format(spec["name"], field)


def test_optional_inputs_are_marked_and_the_rest_are_not(document, specs):
    offered = capabilities.offered(document)
    for spec in specs:
        for field, meta in spec["args"].items():
            declared = offered[spec["name"]]["inputs"][field]
            assert meta["optional"] == bool(declared.get("optional")), "{}.{}".format(spec["name"], field)


def test_optional_inputs_can_be_left_out_and_are_not_sent():
    """Left out, an optional input must not reach bt_server at all, not even as null."""
    pytest.importorskip("langchain_core")
    from thorp_agent import tools
    spec = {"name": "t", "description": "a test tool with one required and one optional input",
            "outputs": [],
            "args": {"a": {"type": "std::string", "optional": False, "description": "a"},
                     "b": {"type": "float", "optional": True, "description": "b"}}}
    sent = {}
    tool = tools.build([spec], lambda name, inputs, **_: sent.update(inputs) or {})[0]
    assert tool.args_schema.model_json_schema()["required"] == ["a"]
    tool.invoke({"a": "x"})
    assert sent == {"a": "x"}


def test_states_reach_the_agent(document, specs):
    """
    The point of declaring states: an agent that cannot read what a capability needs, what it
    leaves behind and what it breaks has no way to tell a precondition it has already met from
    one it still has to.
    """
    offered = capabilities.offered(document)
    by_state = capabilities.establishers(document)
    assert by_state, "no capability establishes any state -- has `needs` gone away?"

    for spec in specs:
        declared = offered[spec["name"]]
        for state in declared.get("needs") or {}:
            assert state in spec["description"], "{} doesn't say it needs {}".format(
                spec["name"], state)
        for state in declared.get("invalidates") or []:
            assert state in spec["description"], "{} doesn't say it breaks {}".format(
                spec["name"], state)
        for state, by in by_state.items():
            if spec["name"] in by:
                assert state in spec["description"], "{} establishes {} and doesn't say so".format(
                    spec["name"], state)


def test_where_an_argument_comes_from_is_shown(document, specs):
    offered = capabilities.offered(document)
    sourced = 0
    for spec in specs:
        for field, meta in (offered[spec["name"]].get("inputs") or {}).items():
            for ref in (meta or {}).get("from") or []:
                sourced += 1
                assert ref in spec["args"][field]["description"], (
                    "{}.{} doesn't say it comes from {}".format(spec["name"], field, ref))
    assert sourced, "no input declares where it comes from"


def test_observation_is_offered_as_a_tool():
    """Not a capability: it runs no tree, so capabilities.yaml has nothing to say about it."""
    pytest.importorskip("langchain_core")
    import json
    from thorp_agent import tools

    tool = tools.observer(lambda: {"holding": "cube 1", "target_in_view": False})
    assert tool.name == "robot_status"
    assert json.loads(tool.invoke({})) == {"holding": "cube 1", "target_in_view": False}


def test_error_codes_get_their_names():
    from thorp_agent import errors

    class ThorpError(object):
        SUCCESS = 1
        INVALID_TARGET_POSE = -210

    class MoveBaseResult(object):
        SUCCESS = 0
        PLAN_FAILURE = 50

    names = errors.table(ThorpError, MoveBaseResult)
    out = errors.annotate({"error": -210, "exe_path_error": 50, "count": 50, "error_x": 3}, names)
    assert out["error_name"] == "INVALID_TARGET_POSE"
    assert out["exe_path_error_name"] == "PLAN_FAILURE"
    assert "count_name" not in out and "error_x_name" not in out
