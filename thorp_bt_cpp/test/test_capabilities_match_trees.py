"""
Checks config/capabilities.yaml still describes the trees it claims to describe.

That file is what the agent reasons from, and hand-written documentation rots quietly: rename
a blackboard key and the yaml goes on promising the old one, so the agent builds goals that
bt_server refuses, and nothing says why until it happens on the robot. So rather than trust
it, this recomputes the same facts from bt/*.xml and fails when the two disagree.

The rules used here are deliberately the ones bt_server applies at runtime:

  an input   is a key some node reads and no node produces. Writing a key back after reading
             it (a bidirectional port, like the list PopPoseFromList pops from) is not
             producing it -- see requiredInputs() in bt_server.cpp, which refuses a goal that
             leaves any of these unset.
  an output  is a key some node writes without reading.

Pure xml and yaml parsing: no roscore, no bt_server, no robot. Run it anywhere, any time.

    cd /catkin_ws/src/thorp/thorp_bt_cpp && python3 -m pytest test/test_capabilities_match_trees.py -v

What it can't check is whether a `description` is *true* -- that a tree said to pick up an
object really does. Those are the fields an LLM leans on hardest, and they still need a human
to read them.
"""

import os
import re
import xml.etree.ElementTree as ET
from glob import glob

import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
BT_DIR = os.path.join(HERE, os.pardir, "bt")
CAPABILITIES = os.path.join(HERE, os.pardir, "config", "capabilities.yaml")
NODE_MODELS = os.path.join(BT_DIR, "node_models.xml")


def normalize_type(declared):
    """
    Strip a C++ type down to something two sources can be compared on: node_models.xml spells
    types out in full, with allocators, char_traits and the pair a map is built from, while
    the yaml writes what a person would. Helper templates are peeled innermost-first and
    repeatedly, so no regex here has to match balanced brackets -- allocator<pair<...>> only
    reduces once the pair inside it already has.
    """
    t = declared.replace("&lt;", "<").replace("&gt;", ">")
    t = re.sub(r"_<std::allocator<void>\s*>", "", t)          # the ros message suffix

    for _ in range(12):
        before = t
        t = re.sub(r"std::__cxx11::basic_string<char(,\s*std::char_traits<char>)?\s*>", "std::string", t)
        t = re.sub(r",?\s*std::allocator<[^<>]*>\s*", "", t)
        t = re.sub(r",?\s*std::less<[^<>]*>\s*", "", t)
        t = re.sub(r",?\s*std::pair<[^<>]*>\s*", "", t)
        if t == before:
            break

    t = re.sub(r"\bunsigned int\b", "uint32_t", t)
    t = re.sub(r"\bunsigned long\b", "uint64_t", t)
    t = re.sub(r"\bunsigned\b", "uint32_t", t)
    t = re.sub(r"(?<!:)\bstring\b", "std::string", t)        # the yaml's shorthand
    t = t.replace("std::std::", "std::")
    t = re.sub(r",\s*>", ">", t)
    return re.sub(r"\s+", "", t)


def port_directions():
    """node id -> port name -> ('in'|'out'|'inout', type), from the generated model file."""
    models = open(NODE_MODELS).read()
    directions = {}
    for match in re.finditer(r'<(Action|Condition|Control|Decorator|SubTree) ID="([^"]+)">(.*?)</\1>',
                             models, re.S):
        node_id, body = match.group(2), match.group(3)
        ports = {}
        for kind, name, type_ in re.findall(
                r'<(input_port|output_port|inout_port)\s+name="([^"]+)"(?:\s+type="([^"]*)")?', body):
            ports[name] = ({"input_port": "in", "output_port": "out", "inout_port": "inout"}[kind],
                           normalize_type(type_ or ""))
        directions[node_id] = ports
    return directions


def remapped_key(port, value):
    """
    The blackboard key a port attribute binds to, or None when it's a literal.

    These are BT.CPP's own rules, from TreeNode::getRemappedKey, which bt_server calls at
    runtime: "{=}" -- or a bare "=" -- is shorthand for a key named after the port, spaces
    around the whole value are ignored while spaces inside the braces belong to the key, and
    "{}" is too short to be a pointer at all. Nothing under bt/ is spelled any of those ways
    today, so this is about the day something is: resolving a spelling differently from the
    server would leave this test checking an interface nobody enforces, which is the one
    failure it has no way to report.
    """
    value = value.strip()
    if value in ("{=}", "="):
        return port
    if len(value) >= 3 and value.startswith("{") and value.endswith("}"):
        return value[1:-1]
    return None


def derive_interfaces():
    """tree id -> {'inputs': {key: type}, 'outputs': {key: type}, 'file': path}."""
    directions = port_directions()
    interfaces = {}

    for path in sorted(glob(os.path.join(BT_DIR, "*.xml"))):
        if os.path.basename(path) == "node_models.xml":
            continue
        for tree in ET.parse(path).getroot().findall("BehaviorTree"):
            read, produced, types = {}, {}, {}
            for node in tree.iter():
                if node.tag == "BehaviorTree":
                    continue
                node_reads, node_writes = set(), set()
                for port, value in node.attrib.items():
                    key = remapped_key(port, value)
                    if not key:
                        continue
                    # a <SubTree> remapping carries no direction here; treat it as a read, so
                    # a key only ever passed down still counts as something to supply
                    direction, type_ = ("in", "") if node.tag == "SubTree" \
                        else directions.get(node.tag, {}).get(port, (None, ""))
                    if direction in ("in", "inout", None):
                        node_reads.add(key)
                        read.setdefault(key, type_)
                    if direction in ("out", "inout"):
                        node_writes.add(key)
                        types.setdefault(key, type_)
                # only a pure write produces a key; a read-then-write hands back what it got
                produced.update({k: types.get(k, "") for k in node_writes - node_reads})

            interfaces[tree.get("ID")] = {
                "inputs": {k: v for k, v in read.items() if k not in produced},
                "outputs": produced,
                "file": os.path.basename(path),
            }
    return interfaces


@pytest.fixture(scope="module")
def declared():
    with open(CAPABILITIES) as f:
        return yaml.safe_load(f)


@pytest.fixture(scope="module")
def derived():
    return derive_interfaces()


def test_every_tree_is_either_described_or_excluded(declared, derived):
    """
    Adding a tree should force a decision: describe it for the agent, or say why it isn't a
    capability. Silence is the failure mode this catches -- a tree nobody documented is a
    capability the agent will never know exists.
    """
    accounted = set(declared["capabilities"]) | set(declared.get("not_capabilities", {}))
    missing = set(derived) - accounted
    assert not missing, (
        "trees in bt/ that capabilities.yaml neither describes nor excludes: {}".format(sorted(missing)))


def test_nothing_is_described_that_does_not_exist(declared, derived):
    """The other direction: an entry left behind after its tree was renamed or deleted."""
    for section in ("capabilities", "not_capabilities"):
        stale = set(declared.get(section) or {}) - set(derived)
        assert not stale, "{} names trees that no longer exist in bt/: {}".format(section, sorted(stale))


def capability_ids(declared_caps):
    return sorted(declared_caps)


def pytest_generate_tests(metafunc):
    if "capability_name" in metafunc.fixturenames:
        with open(CAPABILITIES) as f:
            caps = yaml.safe_load(f)["capabilities"]
        metafunc.parametrize("capability_name", sorted(caps))


def test_declared_inputs_match_the_tree(capability_name, declared, derived):
    """
    Exactly, not loosely: bt_server refuses a goal missing any required input, so an entry
    promising too few has the agent building goals that always bounce, and one promising too
    many has it sending keys nothing reads.
    """
    spec = declared["capabilities"][capability_name]
    assert capability_name in derived, "no such tree"

    declared_inputs = set(spec.get("inputs") or {})
    derived_inputs = set(derived[capability_name]["inputs"])
    assert declared_inputs == derived_inputs, (
        "{}: yaml says inputs {} but the tree needs {}".format(
            capability_name, sorted(declared_inputs), sorted(derived_inputs)))


def test_declared_outputs_exist_in_the_tree(capability_name, declared, derived):
    """
    Subset, not equality: a tree may write a dozen intermediate keys and the yaml documents
    the few worth handing an agent. What it must never do is promise one the tree never sets.
    """
    spec = declared["capabilities"][capability_name]
    declared_outputs = set(spec.get("outputs") or {})
    derived_outputs = set(derived[capability_name]["outputs"])

    invented = declared_outputs - derived_outputs
    assert not invented, (
        "{}: yaml promises outputs the tree never writes: {}".format(capability_name, sorted(invented)))


def test_declared_types_match_the_ports(capability_name, declared, derived):
    """Types come from the ports themselves, so a retyped port shows up here as a mismatch."""
    spec = declared["capabilities"][capability_name]
    interface = derived[capability_name]

    for section in ("inputs", "outputs"):
        for key, entry in (spec.get(section) or {}).items():
            expected = interface[section].get(key, "")
            if not expected:
                continue  # port has no declared type in node_models.xml; nothing to compare
            assert normalize_type(entry.get("type", "")) == expected, (
                "{}.{}.{}: yaml says {!r}, the port is {!r}".format(
                    capability_name, section, key, entry.get("type"), expected))


def test_setup_steps_are_callable_capabilities(capability_name, declared):
    """
    A `setup` step is a real goal the test suite will send, and `inputs_from` is a real lookup
    into an earlier step's result. Both fail quietly: an unknown subtree is refused by
    bt_server and a bad reference raises before the goal is sent, and either way the test
    skips with "precondition not established" -- which reads exactly like the robot not
    cooperating. Checking it here, where nothing needs to be running, keeps that from going
    unnoticed for weeks.
    """
    caps = declared["capabilities"]
    spec = caps[capability_name]
    test = spec.get("test") or {}
    steps = test.get("setup") or []

    seen = {}   # label -> the outputs that step will have produced by the time it is done
    for step in steps:
        target = step.get("subtree")
        assert target in caps, (
            "{}: setup calls '{}', which is not a capability".format(capability_name, target))
        check_call(capability_name, caps[target], step, seen,
                   "setup step '{}'".format(step.get("as") or target))
        seen[step.get("as") or target] = set(caps[target].get("outputs") or {})

    check_call(capability_name, spec, test, seen, "the capability itself")

    declared_outputs = set(spec.get("outputs") or {})
    for key in test.get("expect_outputs") or []:
        assert key in declared_outputs, (
            "{}: expect_outputs names {}, which isn't a declared output".format(
                capability_name, key))


def check_call(capability_name, target_spec, block, available, what):
    """One call's inputs: every input supplied exactly once, and every reference resolvable."""
    literal = set(block.get("inputs") or {})
    threaded = set(block.get("inputs_from") or {})

    overlap = literal & threaded
    assert not overlap, "{}: {} gives {} both literally and via inputs_from".format(
        capability_name, what, sorted(overlap))

    required = set(target_spec.get("inputs") or {})
    assert literal | threaded == required, (
        "{}: {} passes {} but that capability takes {}".format(
            capability_name, what, sorted(literal | threaded), sorted(required)))

    for key, ref in (block.get("inputs_from") or {}).items():
        assert isinstance(ref, list) and len(ref) in (2, 3), (
            "{}: {} has a malformed inputs_from for {}: {!r}".format(
                capability_name, what, key, ref))
        step_label, out_key = ref[0], ref[1]
        assert step_label in available, (
            "{}: {} takes {} from '{}', which is not an earlier setup step (have: {})".format(
                capability_name, what, key, step_label, sorted(available) or "none"))
        assert out_key in available[step_label], (
            "{}: {} takes {} from {}.{}, which that capability doesn't declare as an "
            "output".format(capability_name, what, key, step_label, out_key))


def root_never_finishes(tree):
    """
    True when a tree's root can't report success on its own: KeepRunningUntilFailure returns
    RUNNING or FAILURE and never SUCCESS, and Repeat with no cycle limit (absent, or -1)
    repeats forever. Asking one of these to "run and tell me how it went" has no answer.
    """
    root = list(tree)[0]
    if root.tag == "KeepRunningUntilFailure":
        return True
    return root.tag == "Repeat" and root.get("num_cycles", "-1") == "-1"


def test_endless_trees_are_marked_as_apps(declared):
    """
    The one half of `kind` that can be derived rather than trusted.

    Whether a terminating tree is a capability or an app is a judgement -- patrol_2_points
    finishes, and is still not something to offer an agent. But a tree that can never finish
    is never a capability: the agent would call it, get nothing back, and be holding a robot
    that has stopped listening. Marking one `kind: capability`, or forgetting to mark it at
    all, is the mistake that matters, so it is checked here instead of remembered.
    """
    endless = set()
    for path in sorted(glob(os.path.join(BT_DIR, "*.xml"))):
        if os.path.basename(path) == "node_models.xml":
            continue
        for tree in ET.parse(path).getroot().findall("BehaviorTree"):
            if root_never_finishes(tree):
                endless.add(tree.get("ID"))

    exposed = {name for name, spec in declared["capabilities"].items()
               if (spec.get("kind") or "capability") != "app"}
    wrongly_offered = endless & exposed
    assert not wrongly_offered, (
        "these trees can never finish, so they can't be capabilities -- mark them `kind: app`: "
        "{}".format(sorted(wrongly_offered)))
