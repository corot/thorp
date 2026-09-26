"""
Checks config/capabilities.yaml still describes the trees it claims to describe.

That file is what the agent reasons from, and hand-written documentation rots quietly: rename
a blackboard key and the yaml goes on promising the old one, so the agent builds goals that
fail on a key it was never told about, and nothing says why until it happens on the robot. So
rather than trust it, this recomputes the same facts from bt/*.xml and fails when the two
disagree:

  an input   is a key some node reads and no node produces. Writing a key back after reading
             it (a bidirectional port, like the list PopPoseFromList pops from) is not
             producing it: the caller still has to supply the list in the first place.
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
                "reads": read,
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
    interface = derived[capability_name]

    inputs = spec.get("inputs") or {}
    required = {k for k, v in inputs.items() if not (v or {}).get("optional")}
    derived_inputs = set(interface["inputs"])
    assert required == derived_inputs, (
        "{}: yaml says inputs {} but the tree needs {}".format(
            capability_name, sorted(required), sorted(derived_inputs)))

    # optional: the tree reads it when given, and produces it itself otherwise
    for key in set(inputs) - required:
        assert key in interface["reads"] and key in interface["outputs"], (
            "{}: {} is marked optional, but the tree doesn't both read it and produce a "
            "fallback for it".format(capability_name, key))


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
            expected = interface[section].get(key) or interface["reads"].get(key, "") \
                if section == "inputs" else interface[section].get(key, "")
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


def establishers(caps):
    """state -> the capabilities that establish it, read off by inverting every `needs`."""
    by_state = {}
    for spec in caps.values():
        for state, capability in (spec.get("needs") or {}).items():
            by_state.setdefault(state, set()).add(capability)
    return by_state


def test_states_are_declared_and_reachable(declared):
    """
    A state is a promise that some capability can put the robot into it. One that nothing
    establishes is a precondition the agent can read and never satisfy, and a name missing from
    the glossary means nothing to it at all.
    """
    caps, states = declared["capabilities"], declared.get("states") or {}

    for name, spec in caps.items():
        for state, by in (spec.get("needs") or {}).items():
            assert state in states, "{} needs '{}', which no state declares".format(name, state)
            assert by in caps, "{} says '{}' comes from '{}', which is not a capability".format(
                name, state, by)
        for state in spec.get("invalidates") or []:
            assert state in states, "{} invalidates '{}', which no state declares".format(name, state)

    unreachable = set(states) - set(establishers(caps))
    assert not unreachable, (
        "states nothing establishes, so nothing can satisfy them: {}".format(sorted(unreachable)))


def test_input_sources_name_real_outputs(capability_name, declared):
    """`from` is what tells the agent where a value comes from; a stale one sends it nowhere."""
    caps = declared["capabilities"]
    for key, entry in (caps[capability_name].get("inputs") or {}).items():
        for ref in (entry or {}).get("from") or []:
            source, _, output = ref.partition(".")
            assert source in caps, "{}.{}: '{}' is not a capability".format(
                capability_name, key, source)
            assert output in (caps[source].get("outputs") or {}), (
                "{}.{}: {} declares no output '{}'".format(capability_name, key, source, output))


def test_declarations_account_for_the_setup_chain(capability_name, declared):
    """
    The setup chains were written by hand against the real robot and they pass, so they are the
    evidence that `needs` and `from` are right. The two are held against each other here rather
    than generated from each other: two accounts of the same knowledge only cross-check while
    both are written down.

    A setup step earns its place one of two ways -- it produces a value a later step consumes,
    or it leaves the robot in a state this capability needs, directly or through one of the
    capabilities that establish those states. place_on_tray needs only a held object, but
    getting one held means a table to pick from. Any other step is one the agent will never
    know to take.
    """
    caps = declared["capabilities"]
    spec = caps[capability_name]
    test = spec.get("test") or {}
    steps = test.get("setup") or []
    if not steps:
        return

    subtree_of = {(step.get("as") or step["subtree"]): step["subtree"] for step in steps}
    needs = spec.get("needs") or {}
    consumed = set()

    for block in steps + [test]:
        # whose inputs this block is filling: a setup step's own, or the capability's
        target = caps[subtree_of.get(block.get("subtree"), capability_name)] \
            if block is not test else spec
        for key, ref in (block.get("inputs_from") or {}).items():
            consumed.add(ref[0])
            sources = ((target.get("inputs") or {}).get(key) or {}).get("from")
            if sources is None:
                continue
            supplied = "{}.{}".format(subtree_of.get(ref[0], ref[0]), ref[1])
            assert supplied in sources, (
                "{}: the test fills {} from {}, which that input's `from` doesn't list "
                "({})".format(capability_name, key, supplied, sources))

    staging = set()
    pending = list(needs.values())
    while pending:
        capability = pending.pop()
        if capability in staging:
            continue
        staging.add(capability)
        pending += list((caps[capability].get("needs") or {}).values())

    for step in steps:
        if (step.get("as") or step["subtree"]) in consumed:
            continue
        assert step["subtree"] in staging, (
            "{}: setup runs '{}' and nothing uses what it returns, so it is there for the state "
            "it leaves behind -- say which, in `needs`".format(capability_name, step["subtree"]))

    staged = {step["subtree"] for step in steps}
    for state, by in needs.items():
        assert by in staged, (
            "{}: needs '{}' from {}, but the setup chain never runs it".format(
                capability_name, state, by))


def check_call(capability_name, target_spec, block, available, what):
    """One call's inputs: every input supplied exactly once, and every reference resolvable."""
    literal = set(block.get("inputs") or {})
    threaded = set(block.get("inputs_from") or {})

    overlap = literal & threaded
    assert not overlap, "{}: {} gives {} both literally and via inputs_from".format(
        capability_name, what, sorted(overlap))

    inputs = target_spec.get("inputs") or {}
    required = {k for k, v in inputs.items() if not (v or {}).get("optional")}
    given = literal | threaded
    assert required <= given <= set(inputs), (
        "{}: {} passes {} but that capability takes {} (optional: {})".format(
            capability_name, what, sorted(given), sorted(required), sorted(set(inputs) - required)))

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

    Whether a terminating tree is a capability or an app is a judgment -- patrol_2_points
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
