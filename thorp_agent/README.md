# thorp_agent

An LLM agent that drives Thorp. It reads `capabilities.yaml`, offers each behavior tree as a
tool, and turns the agent's calls into `RunSubtree` goals. Built on ROSA.

## How it works

`thorp_bt_cpp/config/capabilities.yaml` is the only source of truth. Its drift test already holds
it to the trees, so a tool offered here is a tree that exists, takes the inputs named and returns
the outputs promised. This package adds the step after: each capability becomes a LangChain tool
whose arguments are the declared inputs and whose description is the declared description plus
what comes back.

Two rules decide what an agent is offered. Anything marked `kind: app` is withheld — `cat_hunter`
and `explore_house` run until canceled, and a model that starts one has no way to get the robot
back. Anything marked `status: blocked` is withheld too, since a tool known not to work only
teaches a model that its tools are unreliable. Twenty capabilities are offered at the moment.

The important thing the agent is told, repeatedly, is that every call builds a fresh tree with an
empty blackboard. Nothing carries over. Picking something off a table is four calls — detect the
table, work out where to stand, approach, detect objects — with the agent passing each result into
the next call itself. Both pose formats and the segmented-object shape round-trip through
`bt_server`'s JSON unchanged, so it can do that by quoting values verbatim.

## Python

`jpl-rosa` needs Python 3.9 or newer, and ROS Noetic runs on 3.8, so `Dockerfile.dev` gives the
agent a venv at `/opt/thorp-agent` on focal's own python3.9, and `nodes/agent.py` names it in its
shebang. ROS's Python packages (`rospy`, `actionlib`, generated messages) are pure Python and come
from the `PYTHONPATH` that `setup.bash` sets. That is also why the node isn't installed with
`catkin_install_python`: its devel relay would run it under 3.8.

## Credentials

OpenRouter by default, so that comparing models is a change of string rather than of code:
`THORP_AGENT_MODEL` picks one (`openai/gpt-5-mini` unless set), and `THORP_AGENT_PROVIDER` can
switch to `anthropic`, `openai` or `ollama` instead. ROSA documents OpenAI, Azure and Ollama but
takes whatever LangChain chat model it's handed, so the others work the same way.

The key comes from `OPENROUTER_API_KEY` and nowhere else, so it never lives in the repo. Export it
on the host, in `~/.bashrc`; `docker-compose.dev.yml` passes it through to the dev container.
Setting a credit limit on the key in OpenRouter caps what a leak can cost.

OpenRouter's free tier allows 50 requests a day, and one chained task costs about seven, so put
a little credit on the account before testing in earnest — it also raises the free-model
allowance to 1000 a day, which is what makes the cheap models worth comparing against.

The default is a reasoning model, which shapes the settings: no temperature, since it accepts
only its own, and no `max_tokens`, since on a reasoning model that caps the hidden reasoning too
and a step that thinks past it loses its tool call. ROSA runs with streaming off, which it needs
to report token usage, and stops after 20 tool calls.

**A Claude.ai subscription does not include API access.** Billed separately, so a
`THORP_AGENT_PROVIDER=anthropic` run needs an `ANTHROPIC_API_KEY` from console.anthropic.com.

## Running it

```
cd /catkin_ws && catkin build thorp_agent && source devel/setup.bash
rosrun thorp_agent agent.py
rosrun thorp_agent agent.py _dry_run:=true
rosrun thorp_agent agent.py _ask:="what can you do?"
```

`_dry_run:=true` prints each goal instead of sending it, so an agent can be watched planning a
chain of calls with only `roscore` running and the robot not moving.

## Tests

`test/` needs neither ROS nor an API key, and skips its one LangChain test when that isn't
installed. It checks the step this package owns: that every offered capability survives the trip
into a tool, that no app leaks through, that every argument type has a rendering, and that a
capability taking a pose tells the model how to write one.

The table a detected surface comes back as is declared field by field rather than left a
free-form object, since it's the value the whole chain turns on: the model gets a shape to copy
instead of a blob to paraphrase. That puts `$ref` in the tool schema, which OpenAI-shaped APIs
accept and Gemini's native function declarations do not — worth knowing before switching
provider.

```
cd /catkin_ws/src/thorp/thorp_agent && python3 -m pytest test/ -v
```

Nothing has run against ROS yet. ROSA itself has been constructed with these tools, with ROS
stubbed out.
