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
    return [capabilities.tool_spec(name, spec)
            for name, spec in sorted(capabilities.offered(document).items())]


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
