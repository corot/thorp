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


def remapped_key(value):
    """The blackboard key a port attribute binds to, or None when it's a literal."""
    value = value.strip()
    return value[1:-1].strip() if value.startswith("{") and value.endswith("}") else None


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
                    key = remapped_key(value)
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
