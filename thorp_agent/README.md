# thorp_agent

An LLM agent that drives Thorp. It reads `capabilities.yaml`, offers each behavior tree as a tool, and turns the
agent's calls into `RunSubtree` goals for `bt_server`. Built on ROSA.

## How it works

`thorp_bt_cpp/config/capabilities.yaml` is the only source of truth. Its drift test already holds it to the trees, so
a tool offered here is a tree that exists, takes the inputs named and returns the outputs promised. This package adds
the step after: each capability becomes a LangChain tool whose arguments are the declared inputs and whose description
is the declared description plus what comes back.

Two rules decide what an agent is offered. Anything marked `kind: app` is withheld: `cat_hunter` and `explore_house`
run until canceled, and a model that starts one has no way to get the robot back. Anything marked `status: blocked` is
withheld too, since a tool known not to work only teaches a model that its tools are unreliable.

The important thing the agent is told, repeatedly, is that every call builds a fresh tree with an empty blackboard.
Nothing carries over. Picking something off a table is several calls (detect the table, approach it, detect objects,
pick one), with the agent passing each result into the next call itself. Poses and detected tables round-trip through
`bt_server`'s JSON unchanged, so it can do that by quoting values verbatim.

Every result carries an `observed` block, read from the robot rather than remembered: what the gripper holds and what
the planning scene contains, from `move_group`, and whether the target tracker sees a target. Whether the gripper is
closed on something isn't observed: only closing it tells.

## Python

ROSA and LangChain have no Jazzy packages. Install them for the system Python, in the user site:

```bash
python3 -m pip install --user --break-system-packages "jpl-rosa[anthropic,ollama]==1.0.10"
```

The PyPI package named `rosa` is an unrelated ROS helper that installs a module of the same name; if it gets
installed after `jpl-rosa`, it empties ROSA's `rosa/__init__.py`, and `from rosa import ROSA` fails. Uninstall it, then
reinstall `jpl-rosa` with `--force-reinstall --no-deps`.

## Credentials

OpenRouter by default, so that comparing models is a change of string rather than of code: `THORP_AGENT_MODEL` picks
one (`openai/gpt-5-mini` unless set), and `THORP_AGENT_PROVIDER` can switch to `anthropic`, `openai` or `ollama`
instead. ROSA takes whatever LangChain chat model it's handed, so the others work the same way.

The key comes from `OPENROUTER_API_KEY` (or `ANTHROPIC_API_KEY`, `OPENAI_API_KEY`) and nowhere else, so it never
lives in the repo. Export it in `~/.bashrc`. Setting a credit limit on the key caps what a leak can cost.

OpenRouter's free tier allows 50 requests a day, and one chained task costs about seven, so put a little credit on the
account before testing in earnest: it also raises the free-model allowance to 1000 a day.

The default is a reasoning model, which shapes the settings: no temperature, since it accepts only its own, and no
`max_tokens`, since on a reasoning model that caps the hidden reasoning too and a step that thinks past it loses its
tool call. ROSA runs with streaming off, which it needs to report token usage, and stops after 20 tool calls.

A Claude.ai subscription does not include API access: a `THORP_AGENT_PROVIDER=anthropic` run needs an
`ANTHROPIC_API_KEY` from console.anthropic.com.

## Running it

Start the LLM playground, the scene the prompts describe: navigation, manipulation, perception and target detection on
the playground world, with a fixed sample of objects on its table and a still cat, and `bt_server` offering all the
trees, none running on its own. Any other app also runs `bt_server` with `executive:=llm`, but on a scene the prompts
don't describe.

```bash
ros2 launch thorp_apps llm_playground.launch.py
```

Then talk to the robot:

```bash
ros2 run thorp_agent agent.py
```

`--ros-args -p dry_run:=true` prints each goal instead of sending it, so an agent can be watched planning a chain of
calls with the robot not moving, and `-p ask:="what can you do?"` asks one question and exits; `agent.launch.py` takes
the same arguments, but gives the node no stdin.

## Tests

`test/` needs neither ROS nor an API key, and skips its LangChain tests when that isn't installed. It checks the step
this package owns: that every offered capability survives the trip into a tool, that no app leaks through, that every
argument type has a rendering, and that a capability taking a pose tells the model how to write one.

The table a detected surface comes back as is declared field by field rather than left a free-form object, since it's
the value the whole chain turns on: the model gets a shape to copy instead of a blob to paraphrase. That puts `$ref` in
the tool schema, which OpenAI-shaped APIs accept and Gemini's native function declarations do not.

```bash
cd ~/colcon_ws/thorp/src/thorp/thorp_agent && python3 -m pytest test/ -v
```
