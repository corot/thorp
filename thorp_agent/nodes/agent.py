#!/opt/thorp-agent/bin/python

"""
A ROSA agent with Thorp's behavior trees as its tools.

Talk to the robot in English; every capability the agent reaches for is a subtree that
bt_server runs. Ctrl-D or 'quit' to leave.

    rosrun thorp_agent agent.py
    rosrun thorp_agent agent.py _dry_run:=true        # show the goals, send nothing
    rosrun thorp_agent agent.py _ask:="pick up cube 3" # one question, then exit
"""

import sys

import rospy

from thorp_agent import capabilities, llm, tools
from thorp_agent.runner import SubtreeRunner, DEFAULT_ACTION


def main():
    rospy.init_node("thorp_agent", anonymous=True)

    path = rospy.get_param("~capabilities", "")
    if not path:
        import rospkg
        path = rospkg.RosPack().get_path("thorp_bt_cpp") + "/config/capabilities.yaml"

    dry_run = rospy.get_param("~dry_run", False)
    specs = capabilities.tool_specs(path)
    rospy.loginfo("Offering %d capabilities: %s",
                  len(specs), ", ".join(spec["name"] for spec in specs))

    runner = SubtreeRunner(action_name=rospy.get_param("~action", DEFAULT_ACTION),
                           dry_run=dry_run)
    # Each capability's own test timeout is what the suite found it needs, so reuse it rather
    # than inventing a number: per_room_coverage wants 900 s and detect_table wants 30.
    document = capabilities.load(path)
    timeouts = {name: (spec.get("test") or {}).get("timeout", 300)
                for name, spec in capabilities.offered(document).items()}

    from rosa import ROSA
    from thorp_agent.prompts import PROMPTS

    agent = ROSA(ros_version=1,
                 llm=llm.make(),
                 tools=tools.build(specs, runner.run, timeouts),
                 prompts=PROMPTS,
                 # ROSA sets the model's streaming itself, and reports token usage only without it
                 streaming=False,
                 show_token_usage=True,
                 # a runaway loop is where the money goes; a whole chain is under ten calls
                 max_iterations=20,
                 verbose=rospy.get_param("~verbose", False))

    question = rospy.get_param("~ask", "")
    if question:
        print(agent.invoke(question))
        return

    print("Thorp is listening{}. Ctrl-D to quit.".format(" (dry run)" if dry_run else ""))
    while not rospy.is_shutdown():
        try:
            said = input("\n> ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if said in ("quit", "exit"):
            break
        if not said:
            continue
        try:
            print(agent.invoke(said))
        except Exception as e:  # a bad key, a rate limit, a tool blowing up mid-chain
            print("agent failed: {}: {}".format(type(e).__name__, e))


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, rospy.ROSInterruptException) as e:
        print(e, file=sys.stderr)
        sys.exit(1)
