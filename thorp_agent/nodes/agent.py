#!/usr/bin/env python3

"""
A ROSA agent with Thorp's behavior trees as its tools.

Talk to the robot in English; every capability the agent reaches for is a subtree that bt_server runs. Ctrl-D or
'quit' to leave.

    ros2 run thorp_agent agent.py
    ros2 run thorp_agent agent.py --ros-args -p dry_run:=true         # show the goals, send nothing
    ros2 run thorp_agent agent.py --ros-args -p ask:='pick up cube 1'  # one question, then exit
"""

import os
import sys
import threading

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor

from thorp_agent import capabilities, llm, tools
from thorp_agent.runner import DEFAULT_ACTION, SubtreeRunner
from thorp_agent.world import World


def main():
    rclpy.init()
    node = rclpy.create_node('thorp_agent')
    # Tools run on this thread, waiting on futures the executor completes on its own; it stops spinning before
    # rclpy shuts down, as a process exiting while it spins aborts
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin)
    spinner.start()
    try:
        talk(node)
    finally:
        executor.shutdown()
        spinner.join()
        node.destroy_node()


def talk(node):
    path = node.declare_parameter('capabilities', '').value or os.path.join(
        get_package_share_directory('thorp_bt_cpp'), 'config', 'capabilities.yaml')
    dry_run = node.declare_parameter('dry_run', False).value
    verbose = node.declare_parameter('verbose', False).value
    question = node.declare_parameter('ask', '').value
    action = node.declare_parameter('action', DEFAULT_ACTION).value

    specs = capabilities.tool_specs(path)
    node.get_logger().info('Offering {} capabilities: {}'.format(len(specs),
                                                                  ', '.join(spec['name'] for spec in specs)))

    world = None if dry_run else World(node)
    runner = SubtreeRunner(node, action_name=action, dry_run=dry_run, world=world)
    # Each capability's own test timeout is what the suite found it needs, so reuse it rather than inventing a
    # number: per_room_coverage wants 900 s and detect_table wants 30
    document = capabilities.load(path)
    timeouts = {name: (spec.get('test') or {}).get('timeout', 300)
                for name, spec in capabilities.offered(document).items()}

    from thorp_agent.agent import ThorpAgent
    from thorp_agent.prompts import PROMPTS

    agent = ThorpAgent(ros_version=2,
                       llm=llm.make(),
                       tools=(tools.build(specs, runner.run, timeouts) +
                              ([tools.observer(world.observe)] if world else [])),
                       prompts=PROMPTS,
                       # ROSA sets the model's streaming itself, and reports token usage only without it
                       streaming=False,
                       show_token_usage=True,
                       # a runaway loop is where the money goes; a whole chain is under ten calls
                       max_iterations=20,
                       verbose=verbose)

    if question:
        print(agent.invoke(question))
        return

    print('Thorp is listening{}. Ctrl-D to quit.'.format(' (dry run)' if dry_run else ''))
    while rclpy.ok():
        try:
            said = input('\n> ').strip()
        except (EOFError, KeyboardInterrupt):
            break
        if said in ('quit', 'exit'):
            break
        if not said:
            continue
        try:
            print(agent.invoke(said))
        except Exception as e:  # a bad key, a rate limit, a tool blowing up mid-chain
            print('agent failed: {}: {}'.format(type(e).__name__, e))


if __name__ == '__main__':
    try:
        main()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError as e:
        print(e, file=sys.stderr)
        sys.exit(1)
    finally:
        if rclpy.ok():
            rclpy.shutdown()
